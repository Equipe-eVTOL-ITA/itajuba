# Configuração de Parâmetros - FSM Fase 4

## Estrutura do arquivo `fsm.yaml`

Este arquivo contém todos os parâmetros utilizados pelos estados da máquina de estados finitos (FSM) da Fase 4.

## Parâmetros por Estado

### 🏠 ArmingState
```yaml
fictual_home_x: 0.0  # Posição X do home fictício
fictual_home_y: 0.0  # Posição Y do home fictício  
fictual_home_z: 0.0  # Posição Z do home fictício
```

### 🚁 TakeoffState / InitialTakeoffState
```yaml
takeoff_height: 1.5          # Altura de decolagem (positiva = para cima)
max_vertical_velocity: 0.5   # Velocidade máxima vertical
position_tolerance: 0.1      # Tolerância de posição para considerar objetivo alcançado
```

### 🛣️ FollowLaneState
```yaml
max_horizontal_velocity: 1.0 # Velocidade máxima horizontal
position_tolerance: 0.1      # Tolerância de posição (reutilizada)

# PID Lateral (Controle de posição X - coordenadas normalizadas)
pid_lateral_kp: 0.5    # Ganho proporcional lateral
pid_lateral_ki: 0.01   # Ganho integral lateral  
pid_lateral_kd: 0.1    # Ganho derivativo lateral

# PID Angular (Controle de orientação theta)
pid_angular_kp: 0.5    # Ganho proporcional angular
pid_angular_ki: 0.01   # Ganho integral angular
pid_angular_kd: 0.1    # Ganho derivativo angular
```

### 🔧 Parâmetros Legacy (compatibilidade)
```yaml
pid_pos_kp: 0.5   # Ganho PID genérico
pid_pos_ki: 0.0   # Ganho PID genérico
pid_pos_kd: 0.1   # Ganho PID genérico
setpoint: 0.0     # Setpoint genérico
```

## Como os Parâmetros São Carregados

1. **Defaults no Código**: Valores padrão definidos em `fase4.cpp`
2. **Arquivo YAML**: Sobrescreve os defaults quando o nó é iniciado
3. **Blackboard**: Parâmetros são colocados no blackboard da FSM
4. **Estados**: Cada estado acessa os parâmetros via `bb.get<T>("parametro")`

## Fluxo de Configuração

```cpp
// 1. Defaults no código
std::map<std::string, std::variant<double, std::string>> defaults = {
    {"takeoff_height", 1.5},
    // ...
};

// 2. ROS2 parameters (do arquivo YAML)
auto parameters = declareAndGetParameters(defaults, ...);

// 3. Blackboard da FSM
fsm_ = std::make_unique<Fase4FSM>(drone, vision, parameters);

// 4. Uso nos estados  
float height = *bb.get<float>("takeoff_height");
```

## Coordenadas Normalizadas

⚠️ **Importante**: Os PIDs laterais trabalham com coordenadas normalizadas (-1 a +1):

- **-1.0**: Extrema esquerda da imagem
- **0.0**: Centro da imagem (objetivo)
- **+1.0**: Extrema direita da imagem

Por isso os ganhos PID laterais são maiores que os tradicionais (que trabalham com pixels).

## Como Ajustar Parâmetros

### Via Launch File
```bash
ros2 launch itajuba_fase4 fase4.launch.py config:=config/fsm.yaml
```

### Via Command Line
```bash
ros2 run itajuba_fase4 fase4_node --ros-args --params-file config/fsm.yaml
```

### Tuning em Runtime
```bash
ros2 param set /fase4_fsm pid_lateral_kp 0.8
```

## Valores Recomendados por Condição

### Ambiente Interno/Controlado
```yaml
pid_lateral_kp: 0.5
pid_lateral_ki: 0.01
pid_lateral_kd: 0.1
```

### Ambiente Externo/Ventoso  
```yaml
pid_lateral_kp: 0.3  # Mais conservador
pid_lateral_ki: 0.02 # Mais integral
pid_lateral_kd: 0.15 # Mais amortecimento
```

### Debug/Teste
```yaml
max_horizontal_velocity: 0.5  # Mais lento
position_tolerance: 0.2       # Mais tolerante
```
