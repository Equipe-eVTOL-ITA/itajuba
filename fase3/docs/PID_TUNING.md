# PID Tuning Guide for Lane Following

## Overview
O sistema de seguimento de faixa utiliza dois controladores PID independentes:

1. **Lateral PID**: Controla o movimento lateral (eixo Y) para manter o drone alinhado com o centro da faixa
2. **Angular PID**: Controla a rotação (yaw) para manter o drone orientado paralelamente à faixa

## Parâmetros ROS2

### Lateral Control (Alinhamento X)
- `pid_lateral_kp`: Ganho proporcional lateral (padrão: 0.01)
- `pid_lateral_ki`: Ganho integral lateral (padrão: 0.001)  
- `pid_lateral_kd`: Ganho derivativo lateral (padrão: 0.005)

### Angular Control (Alinhamento Theta)
- `pid_angular_kp`: Ganho proporcional angular (padrão: 0.5)
- `pid_angular_ki`: Ganho integral angular (padrão: 0.01)
- `pid_angular_kd`: Ganho derivativo angular (padrão: 0.1)

## Como Ajustar

### Lateral PID (Correção de Posição X)
1. **Kp muito baixo**: Drone não corrige desvios laterais rapidamente
2. **Kp muito alto**: Drone oscila de um lado para outro
3. **Ki**: Ajuda a eliminar erro constante (bias), use valores pequenos
4. **Kd**: Reduz oscilações, mas pode amplificar ruído

### Angular PID (Correção de Orientação)
1. **Kp muito baixo**: Drone não se alinha com a direção da faixa
2. **Kp muito alto**: Drone gira muito bruscamente
3. **Ki**: Elimina erro de orientação constante
4. **Kd**: Suaviza a rotação

## Processo de Ajuste
1. Comece com Kp baixo, Ki=0, Kd=0
2. Aumente Kp até obter resposta adequada
3. Adicione Kd para reduzir oscilações
4. Adicione Ki apenas se houver erro constante

## Exemplo de Launch com Parâmetros
```bash
ros2 run itajuba_fase4 fase4_node \
  --ros-args \
  -p pid_lateral_kp:=0.02 \
  -p pid_lateral_ki:=0.001 \
  -p pid_lateral_kd:=0.01 \
  -p pid_angular_kp:=0.8 \
  -p pid_angular_ki:=0.02 \
  -p pid_angular_kd:=0.15
```
