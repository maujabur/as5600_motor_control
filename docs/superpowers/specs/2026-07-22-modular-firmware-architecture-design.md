# Arquitetura modular do firmware de controle do motor

## Objetivo

Refatorar o firmware para separar responsabilidades por competência, reduzir o
acoplamento e tornar os componentes compreensíveis e testáveis isoladamente.
O painel web, os contratos HTTP e o comportamento físico do equipamento devem
permanecer equivalentes.

A refatoração pode remover código legado, estados redundantes e abstrações que
não participem do comportamento externo desejado. Não há requisito de
compatibilidade com dados já gravados na NVS. Depois da atualização, o
equipamento poderá ser configurado novamente uma única vez.

## Escopo externo preservado

- Aparência, fluxo e recursos das duas páginas web atuais.
- URLs, métodos, campos de entrada e campos JSON usados pelo frontend atual.
- Passo inicial seguido de 1 a 16 passos cíclicos.
- Posição, RPM, espera e direção independentes por passo.
- Movimento avulso pelo menor caminho.
- Continuidade entre passos consecutivos compatíveis.
- STOP imediato durante movimento ou espera.
- Controle ADRC, perfil de velocidade, kick, janela de chegada e stall.
- Bloqueio imediato da ponte H na perda do sensor.
- Retomada do movimento pendente após recuperação do sensor.
- Parada do motor no início de uma transferência OTA.
- Wi-Fi em modo estação com AP de contingência.
- Persistência da configuração e do estado `running` no novo formato.
- Logs relevantes de inicialização, conectividade, falhas e movimento.

## Simplificações autorizadas

- Remover o console serial interativo; a serial ficará restrita a logs e
  diagnóstico.
- Remover `RepetitiveMotionController` e seu modelo paralelo de configuração.
- Remover a máquina de controle PWM em malha aberta e os comandos de teste
  associados.
- Remover a migração da configuração histórica de ida e volta.
- Substituir as chaves históricas da NVS por um novo formato versionado.
- Consolidar enumerações de direção e funções matemáticas duplicadas.
- Remover adaptadores baseados em funções globais e estados usados apenas pelo
  código legado.

## Estrutura proposta

```text
src/
  main.cpp
  MotorControlApplication.h/.cpp

lib/
  domain/
    AngleMath.h/.cpp
    MotionTypes.h

  hardware/
    HBridgeMotorDriver.h/.cpp
    AngleSensor.h
    AngleSensorManager.h/.cpp
    As5600Sensor.h/.cpp
    Mt6701Sensor.h/.cpp

  control/
    VelocityEstimator.h/.cpp
    AdrcPositionController.h/.cpp
    MotionCoordinator.h/.cpp

  sequence/
    MotionSequenceController.h/.cpp

  settings/
    DeviceSettings.h
    PreferencesSettingsStore.h/.cpp

  web/
    WebControlServer.h/.cpp
    repetitive_motion_web_page.h
    control_settings_web_page.h

  connectivity/
    NetworkServices.h/.cpp

  diagnostics/
    MotionTelemetry.h/.cpp
```

PlatformIO trata cada diretório imediato de `lib/` como uma biblioteca. Cada
módulo deverá declarar suas dependências por headers públicos, sem incluir
arquivos internos de outro módulo por caminhos relativos. Caso a resolução de
bibliotecas do PlatformIO exija uma organização diferente, os mesmos limites
lógicos serão mantidos sob uma única biblioteca com subdiretórios.

## Responsabilidades dos módulos

### Domínio

`AngleMath` oferece a única implementação de:

- normalização para uma volta;
- diferença angular pelo menor caminho;
- diferença forçada no sentido horário ou anti-horário;
- atualização de posição angular acumulada;
- cálculo do alvo acumulado para uma direção solicitada.

`MotionTypes` define `MotionDirection`, `MotionStep`,
`MotionSequenceConfig` e os limites compartilhados. Esses tipos não dependem de
Arduino.

### Hardware

`HBridgeMotorDriver` é o único componente autorizado a configurar ou escrever
nos pinos IN1 a IN4 e nas saídas PWM. Sua interface cobre inicialização,
frequência PWM, limite global de potência, saída assinada, coast, brake e parada
segura. O driver sempre inicia com saída zero.

Os sensores e `AngleSensorManager` permanecem responsáveis por I2C, detecção,
leituras, contagem de falhas e redetecção. Eles não controlam movimento nem
saída PWM.

### Controle

`AdrcPositionController` continua responsável pelo controle de posição, perfil
de referência, estimativa de velocidade, kick, critério de chegada e detecção
de stall. Ele recebe ângulo e tempo e devolve uma porcentagem de comando; não
conhece pinos, web, NVS ou sequência.

`MotionCoordinator` integra sensor, ADRC e driver físico. Ele é a fachada para
iniciar, redirecionar e cancelar movimentos. Também concentra a política de
perda e recuperação do sensor, parada segura e publicação de `MotionStatus`.

### Sequência

`MotionSequenceController` é uma máquina de estados independente de Arduino e
hardware. Ele controla os estados parado, movendo e aguardando, seleciona o
próximo passo e solicita operações por uma interface explícita de movimento.

### Configuração

`DeviceSettings` reúne as configurações de posição, sequência, driver, sensor e
o estado persistente `running`.

`PreferencesSettingsStore` é o único componente que conhece o namespace, a
versão e as chaves da NVS. Ele oferece abertura, carga e gravação de um retrato
coerente da configuração. Dados ausentes, incompatíveis ou inválidos produzem
defaults seguros; não haverá migração da versão anterior.

### Web e conectividade

`WebControlServer` registra rotas, valida entradas e produz JSON. Seus handlers
usam uma interface da aplicação e não acessam objetos globais de hardware. As
páginas HTML atuais serão preservadas e apenas realocadas para o módulo web.

`NetworkServices` concentra Wi-Fi, AP de contingência, botão auxiliar e OTA. Ao
receber o evento de início da OTA, solicita parada imediata à aplicação antes
de permitir a atualização.

### Diagnóstico e composição

`MotionTelemetry` coleta métricas de movimento e emite logs periódicos. Ele não
participa das decisões de controle.

`MotorControlApplication` constrói e coordena os módulos. Ele expõe comandos e
retratos de estado usados pelo servidor web. `main.cpp` apenas instancia a
aplicação e encaminha `setup()` e `loop()`.

## Direção das dependências

```text
main -> aplicação -> web / rede / configuração
                 -> sequência -> coordenador -> ADRC -> domínio angular
                                          \-> sensor / driver físico
```

Módulos de domínio, sequência e cálculo não dependem de Arduino. Dependências
específicas do ESP32 ficam nas bordas de hardware, NVS, web e conectividade.
O servidor web observa a aplicação por comandos e snapshots, sem alcançar
diretamente os controladores internos.

## Fluxo de execução

`loop()` chama somente `application.update(millis())`. A aplicação mantém a
seguinte ordem lógica:

1. atender web e OTA;
2. atualizar detecção e recuperação do sensor;
3. atualizar o sequenciador;
4. executar o controle de posição;
5. aplicar a saída física;
6. emitir telemetria quando necessário.

A recuperação do sensor permanece antes da sequência e do controle, impedindo
PWM com uma leitura inválida. A saída física é aplicada somente pelo
coordenador por meio do driver.

## Movimentos e falhas

`MotionCoordinator` expõe operações equivalentes a:

```cpp
startMove(request)
retargetMove(request)
cancelMove()
update(now_ms)
status()
```

O coordenador converte alvos circulares em trajetórias acumuladas com
`AngleMath`. Na perda do sensor, ele mantém a intenção lógica do movimento,
zera imediatamente o driver e congela o avanço temporal da sequência. Quando o
sensor retorna, reancora o controlador no ângulo confirmado, reinicia os
estados dinâmicos necessários e retoma o mesmo movimento.

STOP e início de OTA cancelam sequência, espera, movimento pendente e retomada
automática. Stall cancela o movimento, desliga o ciclo persistente e mantém a
saída em zero. Estados inválidos ou falhas internas do controle também resultam
em parada segura.

## Configuração e validação

O modelo persistido contém uma versão explícita e um retrato completo de
`DeviceSettings`. A validação ocorre antes de aplicar dados a qualquer
controlador. Valores fora dos limites rejeitam a alteração web; dados inválidos
lidos da NVS são substituídos por defaults seguros.

A configuração só pode ser alterada quando sequência, movimento e OTA estiverem
parados. A gravação ocorre depois que o novo retrato for validado e aplicado.
Uma falha de gravação é informada ao cliente sem deixar os módulos com versões
parciais diferentes da configuração.

## Contrato web

As rotas, métodos, nomes de campos e formato esperado pelo JavaScript existente
serão preservados. A aparência das páginas não será redesenhada. Mudanças em
CSS ou JavaScript ficam limitadas a ajustes necessários para manter o contrato
durante a realocação.

Entradas sintaticamente inválidas retornam HTTP 400. Operações válidas, mas
proibidas pelo estado atual, retornam HTTP 409. Falhas internas, incluindo
persistência, retornam HTTP 500. O estado HTTP e o JSON são produzidos a partir
de snapshots imutáveis da aplicação.

## Estratégia de testes

Os testes atuais baseados em busca textual em `main.cpp` serão substituídos ou
adaptados para os contratos públicos dos módulos. A cobertura prioritária é:

- normalização e diferenças nas bordas de 0 e 360 graus;
- direção horária, anti-horária e menor caminho;
- desenrolamento de múltiplas voltas;
- avanço, espera, repetição e continuidade dos passos;
- STOP durante movimento e espera;
- perda e recuperação do sensor;
- stall e bloqueio da ponte H;
- validação e defaults da configuração;
- preservação das rotas e dos campos JSON;
- ordem de atualização da aplicação;
- compilação integral com PlatformIO.

Sempre que possível, domínio e máquinas de estado terão testes nativos sem
Arduino. As integrações específicas do ESP32 serão verificadas por contratos de
fonte e pelo build do firmware.

## Ordem de implementação

1. Criar `AngleMath` e `MotionTypes` com testes.
2. Migrar estimador, ADRC e sequenciador para os tipos compartilhados.
3. Extrair `HBridgeMotorDriver`.
4. Criar `MotionCoordinator` e migrar perda/recuperação do sensor.
5. Consolidar `DeviceSettings` e `PreferencesSettingsStore`.
6. Extrair `WebControlServer` preservando o contrato HTTP.
7. Extrair `NetworkServices` e o fluxo OTA.
8. Criar `MotionTelemetry` e manter apenas logs úteis.
9. Compor tudo em `MotorControlApplication` e reduzir `main.cpp`.
10. Remover módulos, estados, adaptadores e testes obsoletos.
11. Executar testes, build completo e revisão final de segurança.

Cada etapa deve deixar o projeto compilável e preservar a parada segura. Não
serão adicionadas funcionalidades nem alterados os valores de sintonia ADRC
como parte desta refatoração.
