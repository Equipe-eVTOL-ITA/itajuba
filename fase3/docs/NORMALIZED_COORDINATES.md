# Sistema de Coordenadas Normalizadas - Lane Detection

## Mudanças Implementadas

### Antes (Sistema de Pixels)
- **Coordenadas**: Pixels absolutos (ex: 0-640 para X, 0-480 para Y)
- **Origem**: Canto superior esquerdo (0,0)
- **Problema**: Dependente da resolução da câmera

### Depois (Sistema Normalizado)
- **Coordenadas**: Valores normalizados entre -1 e +1
- **Origem**: Centro da imagem (0,0)
- **Vantagem**: Independente da resolução da câmera

## Conversão de Coordenadas

### No Python (lane_detector.py)
```python
# Coordenadas em pixels (sistema original)
cx_pixels = int(M['m10']/A)
cy_pixels = int(M['m01']/A)

# Normalização para centro da imagem
center_x = image_width / 2.0
center_y = image_height / 2.0

cx = (cx_pixels - center_x) / center_x  # -1 a +1
cy = (cy_pixels - center_y) / center_y  # -1 a +1

# Publicação (multiplicado por 1000 para manter precisão como int)
lane_msg.x_centroid = int(cx * 1000)
lane_msg.y_centroid = int(cy * 1000)
```

### No C++ (follow_lane_state.hpp)
```cpp
// Conversão de volta para float
float x_centroid_normalized = static_cast<float>(x_centroid_scaled) / 1000.0f;

// Setpoint agora é 0 (centro da imagem)
static constexpr float TARGET_X_NORMALIZED = 0.0f;
```

## Sistema de Coordenadas Visual

```
(-1,-1) ─────────────── (1,-1)
   │                      │
   │      (0,0)          │    ← Centro da imagem
   │        ●            │
   │                      │
(-1,1) ─────────────── (1,1)
```

## Interpretação dos Valores

### Coordenada X (Lateral)
- **-1.0**: Extrema esquerda da imagem
- **0.0**: Centro da imagem (alvo ideal)
- **+1.0**: Extrema direita da imagem

### Coordenada Y (Vertical)
- **-1.0**: Topo da imagem
- **0.0**: Centro vertical da imagem
- **+1.0**: Base da imagem

## Vantagens do Sistema Normalizado

1. **Independência de Resolução**: Funciona com qualquer tamanho de imagem
2. **PID Mais Intuitivo**: Setpoint sempre é 0 (centro)
3. **Escalabilidade**: Fácil de ajustar para diferentes câmeras
4. **Debug Simplificado**: Valores entre -1 e +1 são mais fáceis de interpretar

## Parâmetros PID Recomendados

```yaml
# Para coordenadas normalizadas (valores menores que pixels)
pid_lateral_kp: 0.5    # Mais alto que antes (era 0.01)
pid_lateral_ki: 0.01   # Proporcional ao novo range
pid_lateral_kd: 0.1    # Ajustado para nova escala
```

## Visualização de Debug

O código agora mostra informações normalizadas na imagem:
- Coordenadas normalizadas (cx, cy)
- Ângulo theta em graus
- Círculo e seta de orientação mantidos em pixels para visualização
