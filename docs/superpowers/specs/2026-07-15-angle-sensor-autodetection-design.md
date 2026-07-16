# Autodetecção de sensores angulares AS5600 e MT6701

## Objetivo

Permitir que o firmware use automaticamente um AS5600 ou MT6701 conectado ao
mesmo barramento I2C, sem seleção manual ou recompilação. O equipamento deve
continuar adequado a uma exposição: se o sensor desaparecer durante um
movimento, o motor deve parar após um limite configurável de falhas, procurar o
sensor periodicamente e retomar com segurança o mesmo movimento quando a
leitura voltar.

Esta primeira versão usa somente I2C nos GPIO 5 (SDA) e 6 (SCL). A saída
analógica/PWM do MT6701 fica fora do escopo.

## Sensores e protocolo

O barramento opera a 400 kHz e aceita exatamente um sensor conectado por vez.

| Sensor | Endereço I2C | Resolução | Registradores de ângulo |
|---|---:|---:|---|
| AS5600 | `0x36` | 12 bits | `0x0C` e `0x0D` |
| MT6701 | `0x06` ou `0x46` | 14 bits | `0x03` e `0x04` |

No MT6701, `0x03` contém `Angle[13:6]` e os seis bits superiores de `0x04`
contêm `Angle[5:0]`. A conversão usa `raw * 360 / 16384`. No AS5600, a
conversão existente permanece `raw * 360 / 4096`.

Referência do MT6701: datasheet oficial MagnTek, revisão 1.8, seção 7.7.2:
<https://www.magntek.com.cn/upload/pdf/202407/MT6701_Rev.1.8.pdf>.

## Arquitetura

### `AngleSensor`

Interface comum para um sensor angular. Expõe:

- nome/modelo;
- endereço I2C ativo;
- sondagem no endereço esperado;
- leitura do valor bruto;
- leitura normalizada em graus.

### Drivers concretos

- `As5600Sensor` mantém isolado o protocolo do AS5600.
- `Mt6701Sensor` implementa o protocolo de 14 bits e aceita `0x06` ou `0x46`.

Os drivers não inicializam o periférico I2C. O barramento é inicializado uma
única vez pelo gerenciador.

### `AngleSensorManager`

Possui instâncias estáticas dos dois drivers, sem alocação dinâmica. É o único
ponto usado por `main.cpp` para detectar sensores, consultar estado e ler
ângulos. Suas responsabilidades são:

- inicializar `Wire` nos GPIO 5/6 a 400 kHz;
- detectar na ordem AS5600 `0x36`, MT6701 `0x06`, MT6701 `0x46`;
- aceitar um sensor somente após resposta no endereço e primeira leitura válida;
- manter modelo, endereço, estado e contador de falhas;
- repetir a detecção a cada 1000 ms quando não houver sensor ativo;
- sinalizar as transições de perda e recuperação para a integração do motor.

`main.cpp` troca todas as referências diretas a `g_as5600` por uma instância
genérica `g_angle_sensor`. Mensagens e APIs deixam de assumir o modelo AS5600.

## Estados e fluxo de dados

O gerenciador usa três estados públicos:

- `DETECTING`: nenhum sensor confirmado e varredura periódica ativa;
- `ACTIVE`: sensor selecionado e leituras disponíveis;
- `LOST`: limite de falhas atingido durante operação; PWM bloqueado enquanto a
  detecção periódica procura um sensor.

No boot, o estado começa em `DETECTING`. Uma leitura de confirmação seleciona o
sensor e muda para `ACTIVE`.

Durante `ACTIVE`, cada leitura válida zera o contador de falhas. Uma leitura
inválida incrementa o contador e o ciclo de controle não recalcula a saída
naquele instante. A saída anterior pode permanecer aplicada apenas até que o
contador alcance o limite; com o padrão de três falhas e controle a 500 Hz,
isso corresponde a no máximo três ciclos de leitura consecutivos.

Ao atingir o limite configurado:

1. o gerenciador muda para `LOST`;
2. a integração aplica PWM zero imediatamente nos dois canais;
3. o destino, sentido, passo e estado `running` são preservados;
4. a detecção volta a ser executada a cada 1000 ms.

## Retomada automática

Quando uma detecção produz uma leitura válida, o gerenciador volta a `ACTIVE`.
Se havia movimento interrompido, o controlador ADRC é retomado sem recriar a
sequência ou avançar o passo. Um novo ponto de integração no controlador deve:

- preservar destino, direção e velocidade máxima do movimento;
- recalcular a posição acumulada a partir do ângulo recuperado;
- reinicializar observador ADRC e estimador de velocidade;
- zerar saída anterior, velocidade de perfil e temporizador de stall;
- reiniciar pela rampa normal de aceleração.

Assim, uma pausa longa não cria falso stall, salto no estimador ou pico de PWM.
A sequência permanece na fase de movimento e continua no mesmo passo. O mesmo
comportamento se aplica ao movimento avulso.

STOP durante `LOST` cancela o controlador e a sequência normalmente. Nesse
caso, uma futura redetecção apenas restaura a leitura angular e não inicia o
motor.

## Configuração persistente

A página `/settings` ganha o campo **Falhas consecutivas do sensor**, com faixa
de 1 a 20 e padrão 3. O valor é salvo em NVS junto aos demais ajustes de
controle e aplicado sem reiniciar o equipamento.

O intervalo de redetecção permanece fixo em 1000 ms nesta versão.

## API, interface e logs

`GET /api/status` preserva o campo booleano `sensor` e acrescenta:

- `sensorType`: `AS5600`, `MT6701` ou `NONE`;
- `sensorAddress`: endereço numérico selecionado, ou 0 sem sensor;
- `sensorState`: `ACTIVE`, `LOST` ou `DETECTING`;
- `sensorFailures`: falhas consecutivas atuais.

Na página principal:

- `ACTIVE` mostra o ângulo normalmente;
- `DETECTING` mostra `DETECTANDO SENSOR`;
- `LOST` mostra `RECONECTANDO SENSOR`;
- a recuperação restaura o ângulo e a indicação do passo sem comando do
  operador.

A serial registra somente transições relevantes: sensor detectado, limite de
falhas atingido e sensor reconectado. Tentativas periódicas que não encontram
sensor não geram spam.

## Tratamento de erros e segurança

- Um endereço I2C que responde mas não fornece a leitura inicial completa não é
  aceito.
- Leituras incompletas ou erros de transmissão contam como falha.
- O contador é consecutivo: qualquer leitura válida o zera.
- Nenhum comando de movimento pode iniciar enquanto o estado não for `ACTIVE`.
- Ao entrar em `LOST`, o PWM zero independe do estado interno da rampa.
- OTA e STOP mantêm precedência e não provocam retomada automática posterior.

## Testes e critérios de aceite

Testes automatizados devem cobrir:

- detecção do AS5600 em `0x36`;
- detecção do MT6701 em `0x06` e `0x46`;
- composição e conversão dos ângulos de 12 e 14 bits;
- boot sem sensor;
- falhas abaixo do limite sem transição para `LOST`;
- leitura válida zerando o contador;
- limite configurado bloqueando o PWM;
- redetecção periódica;
- retomada do mesmo destino, sentido e passo com estado interno reinicializado;
- STOP durante perda impedindo retomada;
- persistência e validação do limite entre 1 e 20;
- compatibilidade dos campos existentes e presença dos novos campos da API;
- textos de estado da interface.

A verificação final executa todos os testes existentes e novos, seguida de
`pio run -e waveshare_esp32_s3_zero`. O README deve documentar os sensores,
endereços, pinagem e recuperação automática.

## Fora do escopo

- uso da saída analógica/PWM, SSI, ABZ ou UVW do MT6701;
- programação de EEPROM ou mudança do endereço do MT6701;
- operação simultânea com dois sensores;
- configuração do intervalo de redetecção;
- retomada de movimento cancelado explicitamente por STOP ou OTA.
