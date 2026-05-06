# ESP32-C3 Joystick ESP-NOW Controller

Sistema de controle remoto usando ESP32-C3 e ESP-NOW, disponível em duas versões para diferentes níveis de complexidade.

## 📁 **Estrutura do Projeto:**

### PlatformIO
- **`src/main.cpp`**: Código unificado (TX/RX) para PlatformIO

### Arduino IDE - Duas Versões Disponíveis
- **`arduino_versions/`**: Códigos separados para Arduino IDE
  
  #### 📁 **Sistema Básico** (para iniciantes)
  - **`transmit/transmit.ino`**: Transmissor básico (1 joystick)
  - **`receive/receive.ino`**: Receptor básico (controle de motores)
  
  #### 🚀 **Sistema Expandido** (para projetos avançados)
  - **`expanded_transmit/expanded_transmit.ino`**: Transmissor expandido (múltiplos sensores)
  - **`expanded_receive/expanded_receive.ino`**: Receptor expandido (controles auxiliares)

### Documentação
- **[README Arduino IDE](arduino_versions/README.md)**: Visão geral e escolha de versão
- **[README Sistema Básico](arduino_versions/README_BASICO.md)**: Guia completo do sistema básico  
- **[README Sistema Expandido](arduino_versions/README_EXPANSION.md)**: Guia do sistema expandido

## 🎯 Qual Versão Escolher?

### Sistema Básico - Para Iniciantes
✅ **Use se:**
- É seu primeiro projeto ESP32
- Quer algo simples e funcional
- Tem apenas 1 joystick
- Precisa apenas de controle de motores

### Sistema Expandido - Para Projetos Avançados  
🚀 **Use se:**
- Quer funcionalidades avançadas (turbo, parada emergência)
- Tem múltiplos sensores (2º joystick, potenciômetro, sensor luz)
- Precisa de controles auxiliares (servo, buzzer, LEDs, relé)
- Está construindo um robô complexo

---

## 🔄 Comparação Rápida

| Funcionalidade | Sistema Básico | Sistema Expandido |
|---|:---:|:---:|
| **Joysticks** | 1 | 2 |
| **Controle velocidade** | ❌ | ✅ Potenciômetro |
| **Modo turbo** | ❌ | ✅ +50% velocidade |
| **Parada emergência** | ❌ | ✅ Botão joystick |
| **Controles auxiliares** | ❌ | ✅ Servo, buzzer, LEDs |
| **Luzes automáticas** | ❌ | ✅ Sensor de luz |
| **Tempo de montagem** | ~30 min | ~2 horas |
| **Ideal para** | Aprendizado | Robô completo |

📖 **Para instruções detalhadas, consulte os READMEs específicos acima!**

---

## Pinos ESP32-C3 Super Mini

### Transmissor (Joystick):
- **GPIO 4**: Eixo X do joystick (analógico)  
- **GPIO 3**: Eixo Y do joystick (analógico)
- **GPIO 2**: Botão do joystick
- **GPIO 8**: LED de status

### Receptor (Motores):
- **GPIO 1-4**: Motores L298N (pinos agrupados)
- **GPIO 8**: LED de status

### Considerações de Boot:
- **GPIO 0**: EVITADO (pino de boot)
- **GPIO 20,21**: EVITADOS (USB UART)  
- **Grupo GPIO 1-4**: Ideal para conexões lado a lado
```
Joystick    ESP32-C3
+5V     ->  3.3V
GND     ->  GND
VRX     ->  GPIO 4
VRY     ->  GPIO 3
SW      ->  GPIO 2
```

## Como usar

### Para PlatformIO (src/main.cpp)

### 1. Descobrir MAC Address para pareamento
**NOVO**: O MAC address agora é exibido automaticamente na inicialização!

#### Para Receptor:
- Configure `MODE RX_MODE` no código
- Faça upload no ESP32 receptor
- Abra o Serial Monitor
- Copie o MAC exibido no formato:
```
*** CONFIGURAÇÃO DE PAREAMENTO ***
MAC Address deste RECEPTOR:
24:6F:28:AA:BB:CC

Configure este MAC no TRANSMISSOR:
uint8_t receiverMAC[] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};
*********************************
```

#### Para Transmissor:
- Configure `MODE TX_MODE` no código
- Cole o MAC do receptor na linha `receiverMAC[]`
- Faça upload no ESP32 transmissor

### Para Arduino IDE (Versões Separadas)

#### Sistema Básico
📖 **Consulte**: [README Sistema Básico](arduino_versions/README_BASICO.md)
- Instruções passo-a-passo completas
- Configuração de hardware e software
- Pinout detalhado e conexões

#### Sistema Expandido  
📖 **Consulte**: [README Sistema Expandido](arduino_versions/README_EXPANSION.md)
- Múltiplos sensores e controles
- Funcionalidades avançadas
- Casos de uso e exemplos práticos

---

### 2. Conectar o joystick no transmissor (pinos otimizados)
```
Joystick    ESP32-C3
GND     ->  GND
+5V     ->  3.3V
VRX     ->  GPIO 4 (eixo X)
VRY     ->  GPIO 3 (eixo Y)  
SW      ->  GPIO 2 (botão)
```

### 3. Conectar a ponte H L298N no receptor (pinos agrupados GPIO 1-4)
```
ESP32-C3 → L298N
Pino 1   → IN1 (PWM+Direção motor esquerdo)
Pino 2   → IN2 (PWM+Direção motor esquerdo)
Pino 3   → IN3 (PWM+Direção motor direito)
Pino 4   → IN4 (PWM+Direção motor direito)
5V       → 5V
GND      → GND
12V      → 12V (alimentação motores)

IMPORTANTE: ENA e ENB devem estar jumpeados (sempre HIGH)
VANTAGEM: Pinos agrupados (1-4) no mesmo lado do módulo
```

### 4. Faça upload nos dispositivos

## Status LEDs
- **3 piscadas**: Inicialização OK
- **LED fixo**: Erro na inicialização
- **Piscadas durante operação**: Dados sendo enviados

## Dados enviados/recebidos
- **joystick_x**: 0-4095 (eixo X)
- **joystick_y**: 0-4095 (eixo Y)
- **button_pressed**: true/false
- **timestamp**: millis() para debugging

## ⚙️ **Configurações Importantes**

### **Limitação de Potência (Proteção de Motores)**
No código (`main.cpp`), você pode ajustar a potência máxima:
```cpp
#define FATOR_DE_POTENCIA 0.75  // 75% da potência máxima
```

**⚠️ CRÍTICO para Motores 5V + Baterias Lítio:**
Se você usa **motores de 5V** com **2 baterias de lítio** (7.4V-8.4V), **SEMPRE** limite a potência para evitar superaquecimento:

```cpp
#define FATOR_DE_POTENCIA 0.6   // Motor 5V protegido com 2S lítio
```

**Explicação:**
- Baterias lítio carregadas: 4.2V × 2 = **8.4V**  
- Motor nominal: **5V**
- Sem limitação: **168% da tensão nominal** = Motor queima! 🔥
- Com 60%: 8.4V × 0.6 = **5.0V** = Seguro! ✅

## Monitor do Receptor
O receptor exibe dados detalhados e comandos de motor no Serial Monitor:
```
[42] Recebido de 24:6F:28:AA:BB:CC:
  RAW    -> X:3072 Y:1024 BTN:FREE
  MAPPED -> X: 255 Y:-128 BTN:FREE REGION:DOWN_RIGHT
  MOTORS -> L:-237 R:   0 EN:YES SPEED:237

[STATUS] Pacotes: 42 | Último: 125 ms | Config X(c:2266 dz:200) Y(c:2221 dz:200)
[TIMEOUT] Sem dados há mais de 1 segundo - PARANDO MOTORES
```

## Tank Drive - Mapeamento de Regiões
- **CENTER**: Motores parados
- **UP**: Ambos motores frente (movimento reto frente)
- **DOWN**: Ambos motores trás (movimento reto trás)
- **LEFT**: Esquerdo trás + Direito frente (giro no lugar esquerda)
- **RIGHT**: Esquerdo frente + Direito trás (giro no lugar direita)
- **UP_LEFT**: Só direito frente (curva suave esquerda)
- **UP_RIGHT**: Só esquerdo frente (curva suave direita)
- **DOWN_LEFT**: Só direito trás (curva suave esquerda em ré)
- **DOWN_RIGHT**: Só esquerdo trás (curva suave direita em ré)

**Proporcionalidade**: Velocidade PWM aplicada diretamente nos pinos IN1-IN4:
- **Motor parado**: IN1=0, IN2=0
- **Motor frente**: IN1=PWM(0-255), IN2=0
- **Motor trás**: IN1=0, IN2=PWM(0-255)

## Hardware L298N
- **ENA/ENB**: Jumpeados (sempre HIGH) 
- **IN1-IN4**: Recebem PWM+direção do ESP32
- **Vantagem**: Usa apenas 4 pinos em vez de 6
- **Desvantagem**: Nenhuma! Funciona perfeitamente

## Segurança
- **Timeout**: Motores param automaticamente se perder comunicação (1s)
- **Inicialização**: Motores iniciados parados
- **Emergency stop**: Região CENTER para parada imediata

## Próximos passos
1. ✅ Sistema completo implementado
2. 🔧 Teste e ajuste de calibração conforme necessário
3. 🎯 Possíveis melhorias: aceleração suave, curve blending

---

## 💡 Resumo de Opções Disponíveis

### 🔧 Para desenvolvedores PlatformIO:
- Use **`src/main.cpp`** (código unificado TX/RX)

### 🎓 Para usuários Arduino IDE:
- **Iniciante**: Use [Sistema Básico](arduino_versions/README_BASICO.md) 
- **Avançado**: Use [Sistema Expandido](arduino_versions/README_EXPANSION.md)

### 📚 Documentação completa:
- **[Visão Geral Arduino](arduino_versions/README.md)**: Escolha de versão
- **[Sistema Básico](arduino_versions/README_BASICO.md)**: Guia completo básico
- **[Sistema Expandido](arduino_versions/README_EXPANSION.md)**: Funcionalidades avançadas

**Dica**: Comece com o sistema básico para entender os conceitos, depois migre para o expandido quando precisar de mais funcionalidades! 🚀