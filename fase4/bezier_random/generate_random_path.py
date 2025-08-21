#!/usr/bin/env python3
"""
Script para gerar caminhos aleatórios em formato SDF para simulação Gazebo
"""

import math
import random
import numpy as np
import argparse
import os
from pathlib import Path

def calculate_direction_change(current_direction, curve_intensity, direction_preference=None, segment_index=0):
    """Calcula a mudança de direção para o próximo segmento

    Args:
        current_direction: Direção atual em radianos
        curve_intensity: Intensidade das curvas
        direction_preference: Preferência direcional
        segment_index: Índice do segmento atual

    Returns:
        Nova direção em radianos
    """
    if direction_preference == 'spiral_left':
        # Espiral no sentido anti-horário
        direction_change = curve_intensity * 0.5
    elif direction_preference == 'spiral_right':
        # Espiral no sentido horário
        direction_change = -curve_intensity * 0.5
    elif direction_preference == 'zigzag':
        # Padrão zigue-zague
        direction_change = curve_intensity * (1 if segment_index % 2 == 0 else -1)
    elif direction_preference == 'return':
        # Tenta voltar na direção oposta gradualmente
        target_direction = current_direction + math.pi
        direction_change = (target_direction - current_direction) * 0.3
    else:
        # Mudança aleatória (padrão)
        direction_change = random.uniform(-curve_intensity, curve_intensity)

    return current_direction + direction_change

def generate_bezier_curve(start, control1, control2, end, num_points=50):
    """Gera pontos ao longo de uma curva de Bézier cúbica"""
    points = []
    for t in np.linspace(0, 1, num_points):
        # Fórmula da curva de Bézier cúbica
        point = (1-t)**3 * start + 3*(1-t)**2*t * control1 + 3*(1-t)*t**2 * control2 + t**3 * end
        points.append(point)
    return points

def generate_smooth_path(start_pos=np.array([0, 0, 0]),
                        total_length=20,
                        num_segments=4,
                        curve_intensity=2.0,
                        initial_direction=None,
                        direction_preference=None):
    """Gera um caminho suave usando múltiplas curvas de Bézier

    Args:
        start_pos: Posição inicial (x, y, z)
        total_length: Comprimento total do caminho
        num_segments: Número de segmentos principais
        curve_intensity: Intensidade das curvas (0-3)
        initial_direction: Direção inicial em radianos (None = aleatória)
                          0 = Norte (+Y), π/2 = Leste (+X), π = Sul (-Y), 3π/2 = Oeste (-X)
        direction_preference: Preferência direcional ('north', 'south', 'east', 'west', 'random', None)
    """

    # Determinar direção inicial
    if initial_direction is not None:
        direction = initial_direction
    elif direction_preference == 'north':
        direction = 0  # +Y
    elif direction_preference == 'east':
        direction = math.pi / 2  # +X
    elif direction_preference == 'south':
        direction = math.pi  # -Y
    elif direction_preference == 'west':
        direction = 3 * math.pi / 2  # -X
    else:
        # Direção inicial aleatória
        direction = random.uniform(0, 2 * math.pi)

    # Dividir o comprimento total em segmentos
    segment_length = total_length / num_segments

    all_points = [start_pos]
    current_pos = start_pos.copy()
    current_direction = direction

    for i in range(num_segments):
        # Ponto final do segmento
        end_x = current_pos[0] + segment_length * math.cos(current_direction)
        end_y = current_pos[1] + segment_length * math.sin(current_direction)
        end_pos = np.array([end_x, end_y, 0.001])  # Ligeiramente acima do chão

        # Pontos de controle para a curva de Bézier
        control_distance = segment_length * 0.3

        # Primeiro ponto de controle (continua a direção atual)
        control1_x = current_pos[0] + control_distance * math.cos(current_direction)
        control1_y = current_pos[1] + control_distance * math.sin(current_direction)
        control1 = np.array([control1_x, control1_y, 0.001])

        # Calcular próxima direção usando a função controlada
        next_direction = calculate_direction_change(
            current_direction, curve_intensity, direction_preference, i
        )

        # Segundo ponto de controle (aponta para a nova direção)
        control2_x = end_pos[0] - control_distance * math.cos(next_direction)
        control2_y = end_pos[1] - control_distance * math.sin(next_direction)
        control2 = np.array([control2_x, control2_y, 0.001])

        # Gerar pontos da curva de Bézier
        curve_points = generate_bezier_curve(current_pos, control1, control2, end_pos, 15)
        all_points.extend(curve_points[1:])  # Pular o primeiro ponto para evitar duplicação

        current_pos = end_pos
        current_direction = next_direction

    return all_points

def create_path_sdf(points, width=0.3, color=[0, 0, 1, 1], model_name="generated_path"):
    """Cria um arquivo SDF com o caminho gerado"""

    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.10">
  <model name="{model_name}">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
"""

    # Criar segmentos do caminho
    for i in range(len(points) - 1):
        start = points[i]
        end = points[i + 1]

        # Calcular posição e orientação do segmento
        mid_x = (start[0] + end[0]) / 2
        mid_y = (start[1] + end[1]) / 2
        mid_z = (start[2] + end[2]) / 2

        # Calcular comprimento e ângulo
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx)

        if length > 0.001:  # Evitar segmentos muito pequenos
            sdf_content += f"""
    <link name="path_segment_{i}">
      <pose>{mid_x} {mid_y} {mid_z} 0 0 {angle}</pose>
      <visual name="path_visual_{i}">
        <geometry>
          <box>
            <size>{length} {width} 0.002</size>
          </box>
        </geometry>
        <material>
          <ambient>{color[0]} {color[1]} {color[2]} {color[3]}</ambient>
          <diffuse>{color[0]} {color[1]} {color[2]} {color[3]}</diffuse>
          <specular>{color[0]} {color[1]} {color[2]} {color[3]}</specular>
        </material>
      </visual>
      <collision name="path_collision_{i}">
        <geometry>
          <box>
            <size>{length} {width} 0.002</size>
          </box>
        </geometry>
      </collision>
    </link>
"""

    # Adicionar plataforma_cruz no final do caminho
    final_point = points[-1]  # Último ponto do caminho
    sdf_content += f"""
    <include>
      <uri>model://sae/plataforma_cruz</uri>
      <pose>{final_point[0]} {final_point[1]} {final_point[2]} 1.57 0 0</pose>
    </include>

  </model>
</sdf>"""

    return sdf_content

def main():
    parser = argparse.ArgumentParser(description='Gera um caminho aleatório para simulação Gazebo')
    parser.add_argument('--length', type=float, default=20.0, help='Comprimento total do caminho')
    parser.add_argument('--width', type=float, default=0.3, help='Largura do caminho')
    parser.add_argument('--segments', type=int, default=4, help='Número de segmentos principais')
    parser.add_argument('--curve-intensity', type=float, default=1.5, help='Intensidade das curvas')
    parser.add_argument('--color', nargs=4, type=float, default=[0, 0, 1, 1], help='Cor RGBA do caminho')
    parser.add_argument('--output', type=str, default='random_path.sdf', help='Arquivo de saída')
    parser.add_argument('--seed', type=int, help='Seed para reproduzir o mesmo caminho')

    # Novos parâmetros de direção
    parser.add_argument('--initial-direction', type=float, help='Direção inicial em graus (0=Norte, 90=Leste, 180=Sul, 270=Oeste)')
    parser.add_argument('--direction-preference', type=str, choices=['north', 'south', 'east', 'west', 'spiral_left', 'spiral_right', 'zigzag', 'return', 'random'],
                       help='Preferência direcional do caminho')

    args = parser.parse_args()

    if args.seed:
        random.seed(args.seed)
        np.random.seed(args.seed)

    # Converter direção inicial de graus para radianos se especificada
    initial_direction = None
    if args.initial_direction is not None:
        # Converter de graus para radianos e ajustar para sistema de coordenadas
        # 0° = Norte (+Y), 90° = Leste (+X), 180° = Sul (-Y), 270° = Oeste (-X)
        initial_direction = math.radians(90 - args.initial_direction)

    # Gerar caminho
    points = generate_smooth_path(
        total_length=args.length,
        num_segments=args.segments,
        curve_intensity=args.curve_intensity,
        initial_direction=initial_direction,
        direction_preference=args.direction_preference
    )

    # Criar SDF
    sdf_content = create_path_sdf(
        points=points,
        width=args.width,
        color=args.color,
        model_name="generated_path"
    )

    # Salvar arquivo
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w') as f:
        f.write(sdf_content)

    print(f"Caminho gerado e salvo em: {output_path}")
    print(f"Pontos no caminho: {len(points)}")
    print(f"Comprimento aproximado: {args.length}m")
    if args.initial_direction is not None:
        print(f"Direção inicial: {args.initial_direction}° ({['Norte', 'Nordeste', 'Leste', 'Sudeste', 'Sul', 'Sudoeste', 'Oeste', 'Noroeste'][int((args.initial_direction + 22.5) % 360 // 45)]})")
    if args.direction_preference:
        print(f"Preferência direcional: {args.direction_preference}")

if __name__ == "__main__":
    main()
