# Motor PWM Tester (Serial) - ESP32-C3 Super Mini

Projeto de bancada para testar motores DC com ponte H L298N via comandos seriais.

Estado atual do firmware:
- Sem Wi-Fi
- Sem ESP-NOW
- Controle por porcentagem de PWM
- Controle de posicao tipo servo com AS5600 (malha fechada)
- Reversao de direcao
- Rampas de aceleracao e desaceleracao ajustaveis
- Dead band configuravel (tstart / tstop)
- Kick de partida e inversao configuravel
- Freio eletrico (curto-circuito regenerativo via L298N)
- Maquina de estados: IDLE / KICK / RUNNING / BRAKE
- Selecao do canal do motor (default em IN3/IN4)

Arquivo principal do firmware:
- src/main.cpp

Modulos reaproveitaveis:
- lib/motion_control/As5600Sensor.h
- lib/motion_control/As5600Sensor.cpp
- lib/motion_control/PositionServoController.h
- lib/motion_control/PositionServoController.cpp

## Objetivo

Permitir testes rapidos e repetiveis de motor, com foco em:
- Ajuste fino de PWM em porcentagem
- Mudanca de sentido (frente/re)
- Ajuste de resposta dinamica (rampas e curva)
- Dead band remapeado para evitar zona morta sem movimento
- Kick de partida para vencer inercial e atrito estatico
- Freio eletrico configuravel (duracao ajustavel)
- Escolha de qual canal da L298N sera comandado

## Hardware alvo

- Placa: ESP32-C3 Super Mini (padrao do projeto)
- Driver: L298N
- Motores: DC escovados (1 ou 2 para teste)

## Mapeamento de pinos

No firmware atual:
- GPIO 1 -> IN1 (motor A)
- GPIO 2 -> IN2 (motor A)
- GPIO 3 -> IN3 (motor B)
- GPIO 4 -> IN4 (motor B)
- GPIO 5 -> I2C SDA (AS5600)
- GPIO 6 -> I2C SCL (AS5600)

Canal padrao ao iniciar:
- IN3/IN4 (motor B)

## Ligacao recomendada (L298N)

Controle:
- ESP32 GPIO1 -> IN1
- ESP32 GPIO2 -> IN2
- ESP32 GPIO3 -> IN3
- ESP32 GPIO4 -> IN4

No modulo L298N:
- ENA e ENB podem ficar jumpeados para este modo de teste
- O PWM e aplicado diretamente nos pinos INx

Alimentacao:
- Fonte dos motores no borne de potencia da L298N
- GND comum entre ESP32 e L298N
- Nao alimente o ESP32 com sobretensao

## Build e upload (PlatformIO)

Ambiente padrao no projeto:
- esp32c3_supermini

Comandos uteis:

```bash
pio run -e esp32c3_supermini
pio run -e esp32c3_supermini -t upload
pio device monitor -b 115200
```

Tambem funciona no VS Code com "Upload" e "Monitor" do PlatformIO.

## Interface serial

Baud rate:
- 115200

Ao iniciar, o firmware imprime ajuda e status, e depois mostra um prompt `>` quando esta pronto para receber comando.

Tambem sao aceitos comandos compactos de letra unica sem espaco antes do primeiro argumento, sem perder suporte aos comandos por extenso. Exemplos:
- `p35`
- `v-40`
- `a250`
- `z80`
- `l60`
- `k100`
- `df`
- `eoff`
- `ma`
- `mb`
- `mm`
- `ts20`
- `kp90`
- `bm300`

Comandos por extenso continuam validos, por exemplo:
- `help`
- `status`
- `motor in3`
- `echo off`
- `motora`
- `motorb`
- `motorboth`

### Comandos disponiveis

Ajuda e estado:

- help | h | ?
  - Mostra lista de comandos

- status | s
  - Mostra estado interno atual

Controle:

- pwm | p <0..100>
  - Define PWM percentual usando a direcao atual
  - Exemplo: pwm 35
  - Forma compacta: p35

- set | v <-100..100>
  - Define velocidade assinada diretamente
  - Exemplo: set -50
  - Forma compacta: v-50

- stop | x
  - Corta a saida imediatamente (IDLE instantaneo, sem rampa)

- brake | b
  - Freio eletrico imediato: aplica IN1=IN2=HIGH no L298N por brakems
  - Cria curto-circuito na bobina (freio regenerativo)
  - Depois do tempo, vai para IDLE

- rev | r
  - Inverte direcao atual e reaplica target

- dir | d f|r
  - Define direcao manual
  - Exemplo: dir f
  - Forma compacta: df

Canal do motor:

- motor | m in3|in1|both
  - Seleciona qual canal comandar
  - in3 = IN3/IN4 (default)
  - in1 = IN1/IN2
  - both = ambos canais

- ma | motora
  - Seleciona diretamente o motor A (IN1/IN2)

- mb | motorb
  - Seleciona diretamente o motor B (IN3/IN4)

- mm | motorboth
  - Seleciona ambos os motores

Rampas e limite:

- accel | a <ms>
  - Tempo de aceleracao
  - Default: 250ms
  - Exemplo: accel 250
  - Forma compacta: a250

- decel | z <ms>
  - Tempo de desaceleracao
  - Default: 80ms
  - Exemplo: decel 80
  - Forma compacta: z80

- curve | c <accel> <decel>
  - Ajusta a curva (gamma) da rampa
  - Default: 1.25 / 1.0
  - Exemplo: curve 1.2 1.0

- limit | l <0..100>
  - Limite global de potencia
  - Default: 100%
  - Exemplo: limit 60
  - Forma compacta: l60

Dead band / kick / freio:

- tstart | ts <0..50>
  - PWM minimo (%) que o motor precisa para comecar a girar
  - O firmware remapeia internamente: qualquer valor >= tstop e enviado no minimo como tstart
  - Default: 20%
  - Forma compacta: ts20

- tstop | tp <0..50>
  - Limiar abaixo do qual o motor e considerado parado
  - Abaixo deste valor a saida PWM vai para 0
  - Deve ser menor que tstart
  - Default: 8%

- kick | k <ms>
  - Duracao do pulso de partida / inversao em milissegundos
  - Enviado sempre que o motor sai do IDLE
  - k0 desabilita o kick
  - Default: 100ms
  - Forma compacta: k100

- kickp | kp <0..100>
  - Potencia do pulso de kick (%)
  - Default: 85%
  - Forma compacta: kp85

- brakems | bm <ms>
  - Duracao do freio eletrico em milissegundos
  - bm0 usa coast (saida zero sem freio)
  - Default: 200ms
  - Forma compacta: bm200

Interface:

- echo | e on|off
  - Liga/desliga o echo dos caracteres digitados no serial
  - Default: on
  - Exemplo: echo off
  - Forma compacta: eoff

- g
  - Le posicao atual do AS5600 (raw e graus)

Posicionamento tipo servo (AS5600):

- q | goto <deg> [rpm]
  - Move para a posicao angular alvo (0..360)
  - rpm max opcional; se omitido usa default interno
  - Exemplo: q 90 6

- qc | cancelmove
  - Cancela movimento de posicao em andamento

- rr | ratedrpm <rpm>
  - Configura RPM nominal do motor para escalar saida percentual
  - Default pensado para seu motor: 18 rpm
  - Exemplo: rr 18

- pk | poskp <rpm/deg>
  - Ganho proporcional do controlador de posicao
  - Exemplo: pk 0.12

- pw | poswin <deg>
  - Janela de erro angular para considerar alvo atingido
  - Exemplo: pw 1.0

## Maquina de estados

O firmware usa 4 fases:

- **IDLE**: motor parado, saida = 0. Ao receber target acima de tstop, passa para KICK (ou direto para RUNNING se kick_ms=0).
- **KICK**: aplica kick_pct durante kick_ms para vencer atrito estatico e inercial. Ao terminar, entra em RUNNING iniciando em tstart.
- **RUNNING**: rampa normal entre current_percent e target_percent. Saida e remapeada pelo dead band.
- **BRAKE**: aplica freio eletrico (IN1=IN2=HIGH) por brake_ms. Ao terminar, volta para IDLE.

Inversao de direcao durante RUNNING:
- O target e invertido, mas current_percent desacelera ate tstop antes de parar.
- Ao atingir tstop, vai para IDLE, que relanca KICK no novo sentido automaticamente.

## Como a rampa funciona

O firmware controla:
- target_percent: alvo definido pelo comando serial
- current_percent: valor aplicado no instante atual

A cada ciclo de controle:
- current_percent se move gradualmente ate target_percent
- accel_ms e usado quando aumentando modulo da velocidade
- decel_ms e usado quando reduzindo modulo da velocidade
- curve accel/decel ajusta o comportamento da transicao

Regra geral:
- Curva maior tende a resposta mais progressiva
- Curva menor tende a resposta mais direta

## Como o dead band funciona

Motores DC tem uma tensao minima para comecar a girar (atrito estatico + dead band do driver).
Abaixo dessa tensao o motor vibra ou nao se move, desperdicando energia.

O firmware remapeia a saida:
- Abaixo de tstop: saida = 0 (motor considerado parado)
- Entre tstop e 100%: remapeado linearmente para [tstart..100%]

Assim qualquer target > tstop resulta em pelo menos tstart de PWM no motor.

Ajuste tstart ate encontrar o valor minimo em que seu motor gira confiavelmente.

## Sequencia de teste sugerida

1. Suba o firmware e abra monitor serial em 115200.
2. Rode `s` para confirmar canal padrao IN3/IN4 e parametros.
3. Comece com `l30` (limite 30%).
4. Aplique `p10` e aumente ate o motor comecar a girar; esse valor e seu tstart real.
5. Ajuste `ts<valor>` com o minimo encontrado acima.
6. Teste `a300` e `z100` para suavidade de rampa.
7. Teste `r` para inversao e observe o kick e a desaceleracao antes de inverter.
8. Use `kp70` e `k80` para afinar o pulso de partida.
9. Teste `b` para freio eletrico e compare com `x` (corte seco).
10. Troque canal com `m in1` se quiser testar IN1/IN2.
11. Use `x` ao finalizar.

## Seguranca

- Sempre inicie com baixa potencia (10% a 30%).
- Use limit para proteger motor e mecanica.
- Garanta fonte adequada para o motor.
- Nunca deixe o motor sem supervisao em bancada.
- Se houver aquecimento excessivo, use stop imediatamente.

## Estrutura minima do projeto

- platformio.ini
- src/main.cpp

## Notas

- Este README descreve somente o estado atual de teste serial.
- Conteudo antigo de projeto doador (joystick/ESP-NOW) foi descontinuado neste firmware.
