# Arquivos Arduino IDE (.ino) - ESP32-C3 Joystick ESP-NOW

Esta pasta contém versões separadas do código para uso no **Arduino IDE**, independente do PlatformIO.

## 📁 **Estrutura de Pastas (Arduino IDE):**

### 1. **`transmit/`** - Pasta do Transmissor (Joystick)
- **Arquivo**: `transmit/transmit.ino` 
- **Função**: Lê joystick analógico e envia dados via ESP-NOW
- **Hardware**: ESP32-C3 Super Mini + Módulo Joystick
- **Pinos**: 
  - GPIO 3: VRX (eixo X)
  - GPIO 4: VRY (eixo Y) 
  - GPIO 2: SW (botão)
  - GPIO 8: LED status

### 2. **`receive/`** - Pasta do Receptor (Motores)
- **Arquivo**: `receive/receive.ino`
- **Função**: Recebe dados ESP-NOW e controla motores L298N
- **Hardware**: ESP32-C3 Super Mini + L298N + 2 Motores DC
- **Pinos**:
  - GPIO 1: Motor IN1 (esquerdo)
  - GPIO 2: Motor IN2 (esquerdo)
  - GPIO 3: Motor IN3 (direito)
  - GPIO 4: Motor IN4 (direito)
  - GPIO 8: LED status

## 🔧 **Como Usar:**

### **Passo 1: Configure o Receptor**
1. Abra a pasta `receive/` no Arduino IDE
2. Abra o arquivo `receive.ino`
3. Selecione placa: **"ESP32C3 Dev Module"**
4. Faça upload no ESP32 do receptor
5. Abra Serial Monitor (115200 baud)
6. **Copie o MAC Address** exibido

### **Passo 2: Configure o Transmissor**
1. Abra a pasta `transmit/` no Arduino IDE
2. Abra o arquivo `transmit.ino`  
2. **Altere a linha:**
   ```cpp
   uint8_t receiverMAC[] = {0x20, 0x6E, 0xF1, 0x6D, 0x9F, 0xBC}; // ALTERE AQUI!
   ```
   Cole o MAC do receptor copiado no Passo 1
3. Faça upload no ESP32 do transmissor

### **Passo 3: Teste**
- Ligue ambos os dispositivos
- Mova o joystick
- Verifique se os motores respondem

## ⚙️ **Configurações importantes:**

### **Limitação de Potência:**
Para reduzir a potência máxima dos motores, altere em ambos os arquivos (`transmit.ino` e `receive.ino`):
```cpp
#define FATOR_DE_POTENCIA 0.75  // 75% da potência máxima
```
**Valores de potência:**
- `0.5` = 50% da potência
- `0.75` = 75% da potência  
- `1.0` = 100% da potência

**⚠️ PROTEÇÃO PARA MOTORES 5V:**
Se você está usando **motores de 5V** com **duas baterias de lítio** (7.4V-8.4V total), é **altamente recomendado** limitar a potência para **0.6-0.7** (60-70%).

**Por quê?**
- Baterias lítio carregadas: 4.2V × 2 = **8.4V**
- Motor nominal: **5V**
- Sobretensão: **8.4V ÷ 5V = 168%** (68% a mais!)
- **Resultado**: Motor superaquece e pode queimar

**Proteção via software:**
```cpp
#define FATOR_DE_POTENCIA 0.6   // Motor 5V + 2S lítio = SEGURO
```
Com 60%, a tensão efetiva fica: 8.4V × 0.6 = **5.0V** ✅

### **Calibração do Joystick:**
No arquivo `receive/receive.ino`, ajuste se necessário:
```cpp
AxisConfig x_axis_config = {
  .center = 2266,           // Valor central X
  .center_deadzone = 200,   // Zona morta (menor = mais sensível)
};

AxisConfig y_axis_config = {
  .center = 2221,           // Valor central Y  
  .center_deadzone = 200,   // Zona morta
};
```

## 🛠️ **Conexões de Hardware:**

### **Transmissor (Joystick):**
```
Joystick → ESP32-C3
GND      → GND  
+5V      → 3.3V
VRX      → GPIO 3
VRY      → GPIO 4
SW       → GPIO 2
```

### **Receptor (Motores):**
```
L298N → ESP32-C3
IN1   → GPIO 1 (motor esquerdo)
IN2   → GPIO 2 (motor esquerdo)
IN3   → GPIO 3 (motor direito) 
IN4   → GPIO 4 (motor direito)
VCC   → 5V
GND   → GND

⚠️ IMPORTANTE: ENA e ENB devem estar jumpeados (sempre HIGH)
```

## 📡 **Características:**

- **Comunicação**: ESP-NOW (baixa latência)
- **Alcance**: ~200m linha de vista
- **Taxa de atualização**: 20Hz (50ms)
- **Potência RF**: Máxima (19.5dBm)
- **Controle**: Tank drive com 9 regiões (incluindo diagonais)
- **Segurança**: Timeout de 1 segundo para parar motores

## 🔍 **Debugging:**

- **LED piscando**: Dados sendo enviados/recebidos
- **LED fixo**: Erro na inicialização
- **3 piscadas no boot**: Inicialização OK
- **Serial Monitor**: Debug detalhado dos comandos

Estes arquivos são **independentes** do PlatformIO e funcionam diretamente no Arduino IDE! 🎯

---

## 📂 **Estrutura de Pastas Arduino IDE:**

```
arduino_versions/
├── README.md              # Este arquivo de instruções
├── transmit/              # Pasta do sketch transmissor
│   └── transmit.ino      # Código do transmissor
└── receive/              # Pasta do sketch receptor  
    └── receive.ino       # Código do receptor
```

**⚠️ IMPORTANTE**: No Arduino IDE, cada sketch (.ino) deve estar em sua própria pasta com o mesmo nome do arquivo. Esta é a convenção padrão do Arduino IDE.