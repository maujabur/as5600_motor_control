# Motor Control — ADRC + AS5600

Firmware para movimento angular repetitivo de um motor DC lento (`rated` 2 RPM),
com realimentação pelo sensor magnético AS5600, controle ADRC, configuração web,
persistência em NVS e atualização OTA.

O equipamento opera de forma autônoma depois de configurado. A serial é usada
somente para logs; comandos recebidos são descartados.

## Funcionalidades

- Sequência configurável com um passo inicial e de 1 a 16 passos cíclicos.
- Posição, RPM, espera e sentido independentes em cada passo.
- Sentido por passo: caminho mais curto, horário (CW) ou anti-horário (CCW).
- Estado `running` persistente entre reinicializações.
- Execução automática do passo 0 quando inicia com `running=on`.
- Movimento avulso para qualquer ângulo pelo caminho mais curto, sem persistência.
- Controle ADRC a 500 Hz, com perfil de aceleração e desaceleração.
- Kick de partida, PWM mínimo, limite de potência e proteção de stall.
- Interface web responsiva para operação e ajustes avançados.
- Wi-Fi cliente com AP automático de contingência.
- OTA e painel web disponíveis simultaneamente com o controle.
- Parada automática do motor quando uma transferência OTA começa.

## Hardware

Placa alvo: **Waveshare ESP32-S3-Zero**.

| Função | GPIO |
|---|---:|
| Ponte H — canal A, IN1 | 1 |
| Ponte H — canal A, IN2 | 2 |
| Ponte H — canal B, IN1 | 3 |
| Ponte H — canal B, IN2 | 4 |
| AS5600 SDA | 5 |
| AS5600 SCL | 6 |
| Botão auxiliar OTA/AP | 7 |

Os dois canais da ponte H são comandados de forma espelhada na configuração
atual. O AS5600 usa I²C Fast Mode em 400 kHz.

Use GND comum entre ESP32, sensor, ponte H e fonte do motor. A alimentação do
motor não deve passar pelo regulador da placa.

## Operação web

### Movimento repetitivo — `/`

- Visualização da posição angular e fase atual.
- RUN e STOP.
- Passo 0 de inicialização, executado uma vez a cada RUN.
- Tabela cíclica variável com até 16 passos.
- Inclusão, exclusão e reordenação de passos quando parado.
- Movimento avulso com posição e RPM, sempre pelo caminho mais curto.

### Ajustes avançados — `/settings`

- ADRC: `wc`, `wo` e `b0`.
- RPM máxima de comando e RPM física estimada.
- Limite global de potência.
- Frequência e PWM mínimo.
- Janela e amostras de chegada.
- Rampas de aceleração e desaceleração.
- Potência e duração do kick.
- Tempo e velocidade para detecção de stall.
- Janela e número de amostras do estimador de velocidade.

Os ajustes avançados só podem ser alterados com o motor parado. Todos os campos
das duas páginas são persistidos na NVS.

## Sequência do ciclo

Ao ativar RUN, o controlador executa o passo 0 e sua espera uma única vez.
Depois percorre os passos 1 até N, aguardando o intervalo configurado após cada
chegada. Ao concluir o passo N, retorna ao passo 1 e repete continuamente.

Todos os passos usam o mesmo formato: posição entre 0 e 359,99°, RPM, intervalo
após a chegada e sentido. A tabela só pode ser alterada quando o ciclo, o servo
e a atualização OTA estão parados. STOP cancela imediatamente movimento ou
espera.

Na primeira inicialização desta versão, uma configuração antiga de vai e volta
é convertida em passo 0 mais dois passos cíclicos. A nova sequência é gravada
como um bloco versionado na NVS ao ser salva pela interface.

STOP cancela imediatamente homing, ajuste pontual ou ciclo automático e salva
`running=off`.

## Wi-Fi

Copie o modelo de credenciais:

```powershell
Copy-Item include/wifi_credentials.example.h include/wifi_credentials.h
```

Edite o arquivo criado:

```cpp
#define WIFI_STA_SSID "SuaRede"
#define WIFI_STA_PASSWORD "SuaSenha"
```

`include/wifi_credentials.h` é ignorado pelo Git.

Na inicialização, o firmware tenta a rede configurada por 12 segundos. Se
conectar, o painel usa o endereço DHCP informado na serial. Caso contrário,
cria o AP de contingência:

- SSID: `Motor-Control-##`
- senha: `12345678`
- painel: `http://192.168.4.1/`
- largura: 20 MHz

O número da unidade é definido em `platformio.ini`:

```ini
-DMOTOR_CONTROL_UNIT=1
```

Para as seis primeiras unidades, os canais são distribuídos sem sobreposição:

| Unidade | SSID | Canal |
|---:|---|---:|
| 1 | `Motor-Control-01` | 1 |
| 2 | `Motor-Control-02` | 6 |
| 3 | `Motor-Control-03` | 11 |
| 4 | `Motor-Control-04` | 1 |
| 5 | `Motor-Control-05` | 6 |
| 6 | `Motor-Control-06` | 11 |

## OTA

- Porta: `3232`
- Senha: `as5600-update`
- Hostname: `as5600-motor-##`

No AP de contingência, o ambiente OTA usa `192.168.4.1`. Quando conectado ao
roteador, informe o IP DHCP da unidade:

```powershell
pio run -e waveshare_esp32_s3_zero_ota -t upload --upload-port 192.168.x.x
```

Uma atualização OTA desativa o motor antes de começar. Após reiniciar, um
`running=on` persistido provoca novo homing, portanto mantenha a mecânica segura.

## Build e upload USB

```powershell
pio run -e waveshare_esp32_s3_zero
pio run -e waveshare_esp32_s3_zero -t upload
pio device monitor -b 115200
```

O ambiente requer o PlatformIO Core gerenciado pelo VS Code, normalmente em:

```text
%USERPROFILE%\.platformio\penv\Scripts\pio.exe
```

## Valores iniciais

| Ajuste | Valor |
|---|---:|
| RPM padrão de ida/volta | 1,0 RPM |
| Pausa no início | 1000 ms |
| Pausa no fim | 2000 ms |
| RPM máxima | 2,4 RPM |
| RPM física estimada | 3,0 RPM |
| ADRC `wc` | 25 |
| ADRC `wo` | 80 |
| ADRC `b0` | 250 |
| Janela de chegada | 1,0° |
| Aceleração | 250 ms |
| Desaceleração | 220 ms |
| Kick | 85% / 180 ms |
| PWM mínimo | 24% |
| PWM carrier | 500 Hz / 8 bits |
| Stall | 2°/s por 1500 ms |
| Estimador de velocidade | 400 ms / 8 amostras |

Valores já gravados na NVS têm prioridade sobre esses defaults.

## Segurança e comissionamento

1. Comece com `running=off` e limite de potência reduzido.
2. Confirme no painel que o AS5600 acompanha o sentido real do eixo.
3. Teste primeiro “IR AO INÍCIO” e “IR AO FIM”.
4. Verifique que CW aumenta e CCW diminui os graus.
5. Ajuste PWM mínimo apenas até vencer o atrito de forma confiável.
6. Valide stall bloqueando o eixo somente em condição mecanicamente segura.
7. Só então habilite o ciclo automático.

Em caso de comportamento inesperado, use STOP pela página e remova a alimentação
de potência da ponte H.

## Estrutura do projeto

```text
include/
  wifi_credentials.example.h
lib/motion_control/
  AdrcPositionController.*
  As5600Sensor.*
  RepetitiveMotionController.*
  VelocityEstimator.*
src/
  main.cpp
  repetitive_motion_web_page.h
  control_settings_web_page.h
platformio.ini
```

`AdrcPositionController` contém o controle e as proteções do motor.
`RepetitiveMotionController` implementa o ciclo sem conhecer hardware.
`main.cpp` integra sensor, ponte H, NVS, Wi-Fi, páginas web e OTA.
