#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fase3 Color Detector — HSV Debug Tool (ROS2 params)

Adaptação do "BaseDetectorDebug" para ler parâmetros de:
fase3_color_detector:
  ros__parameters:
    <chaves globais...>
    <formas>:
      hue_lower: ...
      hue_upper: ...
      saturation_lower: ...
      saturation_upper: ...
      value_lower: ...
      value_upper: ...

Uso:
    python3 fase3_color_detector_debug.py \
        --params /home/ceccon/frtl_2025_ws/src/itajuba/fase3/config/onboard.yaml
        [--image /caminho/para/imagem.jpg] \
        [--output /caminho/saida] \
        [--min_ar 0.7 --max_ar 1.3 --max_area 120000]

Saídas:
- PNG com faixas HSV por forma
- TXT com parâmetros carregados
- (Opcional) PNG com debug de detecções na imagem e grade de máscaras
"""

import cv2
import os
import yaml
import argparse
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, Tuple, List


def _mk_color(i: int) -> Tuple[int, int, int]:
    """Gera uma cor BGR estável por índice."""
    # Cores espaçadas no círculo HSV para desenhar no BGR
    hue = (i * 37) % 180
    chip = np.uint8([[[hue, 200, 255]]])  # HSV
    bgr = cv2.cvtColor(chip, cv2.COLOR_HSV2BGR)[0, 0].tolist()
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


class Fase3ColorDetectorDebug:
    def __init__(self, params_file: str = None,
                 min_ar: float = 0.6, max_ar: float = 1.8,
                 min_area: int = 500, max_area: int = 200000):
        self.script_dir = os.path.dirname(os.path.realpath(__file__))

        # Parâmetros gerais (defaults)
        self.general = {
            'morph_kernel_size': 5,
            'morph_iterations': 2,
            'min_pad_area': int(min_area),
            'max_pad_area': int(max_area),
            'min_aspect_ratio': float(min_ar),
            'max_aspect_ratio': float(max_ar),
        }

        # Formas carregadas do YAML: { nome: {hue_lower, hue_upper, ...} }
        self.shapes: Dict[str, Dict[str, int]] = {}

        if params_file and os.path.exists(params_file):
            self._load_from_yaml(params_file)
        else:
            print("[AVISO] Arquivo de parâmetros não informado/encontrado. "
                  "Rodando apenas com defaults e sem formas.")

    # ---------------- YAML ----------------

    def _load_from_yaml(self, params_file: str):
        try:
            with open(params_file, 'r') as f:
                data = yaml.safe_load(f) or {}

            # Esperado: data['fase3_color_detector']['ros__parameters']
            f3 = data.get('fase3_color_detector', {})
            ros_params = f3.get('ros__parameters', {})

            # Mapeia área mínima (nome do seu YAML)
            if 'area_minima_bases' in ros_params:
                self.general['min_pad_area'] = int(ros_params['area_minima_bases'])

            # Procura sub-blocos que sejam dicionários com chaves HSV
            hsv_keys = {'hue_lower', 'hue_upper', 'saturation_lower', 'saturation_upper',
                        'value_lower', 'value_upper'}
            loaded = 0
            for k, v in ros_params.items():
                if isinstance(v, dict) and hsv_keys.issubset(set(v.keys())):
                    # Garante int
                    self.shapes[k] = {
                        'hue_lower': int(v['hue_lower']),
                        'hue_upper': int(v['hue_upper']),
                        'saturation_lower': int(v['saturation_lower']),
                        'saturation_upper': int(v['saturation_upper']),
                        'value_lower': int(v['value_lower']),
                        'value_upper': int(v['value_upper']),
                    }
                    loaded += 1

            print(f"[OK] Carregado do YAML: {loaded} forma(s).")
            print(f"[INFO] min_pad_area = {self.general['min_pad_area']} "
                  f"(de 'area_minima_bases' se presente)")

        except Exception as e:
            print(f"[ERRO] Falha ao carregar YAML: {e}")

    # ---------------- Segmentação ----------------

    def _mask_from_range(self, hsv_img: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
        """
        Cria máscara binária considerando wrap-around em H (0..180 OpenCV).
        Se lo[0] <= hi[0]: mascara direta.
        Senão: junta [loH..180] e [0..hiH].
        """
        h_lo, s_lo, v_lo = int(lo[0]), int(lo[1]), int(lo[2])
        h_hi, s_hi, v_hi = int(hi[0]), int(hi[1]), int(hi[2])

        # Clampa intervalos para segurança
        h_lo = max(0, min(180, h_lo))
        h_hi = max(0, min(180, h_hi))
        s_lo = max(0, min(255, s_lo))
        s_hi = max(0, min(255, s_hi))
        v_lo = max(0, min(255, v_lo))
        v_hi = max(0, min(255, v_hi))

        if h_lo <= h_hi:
            mask = cv2.inRange(hsv_img, np.array([h_lo, s_lo, v_lo]),
                               np.array([h_hi, s_hi, v_hi]))
        else:
            # wrap-around
            m1 = cv2.inRange(hsv_img, np.array([h_lo, s_lo, v_lo]),
                             np.array([180, s_hi, v_hi]))
            m2 = cv2.inRange(hsv_img, np.array([0, s_lo, v_lo]),
                             np.array([h_hi, s_hi, v_hi]))
            mask = cv2.bitwise_or(m1, m2)
        return mask

    def segment_shape(self, hsv_img: np.ndarray, shape_name: str) -> np.ndarray:
        """Cria máscara para uma forma do YAML."""
        if shape_name not in self.shapes:
            return np.zeros(hsv_img.shape[:2], dtype=np.uint8)
        p = self.shapes[shape_name]
        lo = np.array([p['hue_lower'], p['saturation_lower'], p['value_lower']])
        hi = np.array([p['hue_upper'], p['saturation_upper'], p['value_upper']])

        mask = self._mask_from_range(hsv_img, lo, hi)

        # Morfologia
        k = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.general['morph_kernel_size'], self.general['morph_kernel_size'])
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k,
                                iterations=self.general['morph_iterations'])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
        return mask

    # ---------------- Detecção ----------------

    def find_candidates(self, image_bgr: np.ndarray) -> Tuple[Dict[str, List[Dict]], np.ndarray, Dict[str, np.ndarray]]:
        """
        Para cada forma, encontra contornos e devolve:
            - dict por forma com lista de candidatos (bbox, área, AR, válido)
            - imagem de debug anotada
            - máscaras por forma
        """
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        debug = image_bgr.copy()
        all_candidates: Dict[str, List[Dict]] = {}
        masks: Dict[str, np.ndarray] = {}

        # Desenha cor única por forma
        name_to_color = {name: _mk_color(i) for i, name in enumerate(sorted(self.shapes.keys()))}

        for name in sorted(self.shapes.keys()):
            mask = self.segment_shape(hsv, name)
            masks[name] = mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            cand_list = []
            for i, c in enumerate(contours):
                x, y, w, h = cv2.boundingRect(c)
                area = float(w * h)
                ar = float(w) / (h + 1e-9)

                area_ok = (self.general['min_pad_area'] <= area <= self.general['max_pad_area'])
                ar_ok = (self.general['min_aspect_ratio'] <= ar <= self.general['max_aspect_ratio'])
                valid = bool(area_ok and ar_ok)

                color = name_to_color[name] if valid else (0, 0, 255)

                # Caixa e rótulo
                cv2.rectangle(debug, (x, y), (x + w, y + h), color, 2)
                label = f"{name} #{i} {'OK' if valid else 'X'}"
                cv2.putText(debug, label, (x, max(0, y - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
                cv2.putText(debug, f"A:{area:.0f} AR:{ar:.2f}",
                            (x, y + h + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

                cand_list.append({
                    'name': name,
                    'bbox': (int(x), int(y), int(w), int(h)),
                    'area': area,
                    'aspect_ratio': ar,
                    'area_ok': area_ok,
                    'ar_ok': ar_ok,
                    'valid': valid,
                })
            all_candidates[name] = cand_list

        # Legenda de cores
        y0 = 22
        for name in sorted(self.shapes.keys()):
            color = name_to_color[name]
            cv2.putText(debug, name, (10, y0),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
            y0 += 22

        return all_candidates, debug, masks

    # ---------------- Visualizações ----------------

    def _plot_color_tile(self, ax, title: str, p: Dict[str, int], samples_per_dim: int = 8):
        h_min, h_max = p['hue_lower'], p['hue_upper']
        s_min, s_max = p['saturation_lower'], p['saturation_upper']
        v_min, v_max = p['value_lower'], p['value_upper']

        # Amostras em SxV com H fixo (meio do intervalo, respeitando wrap)
        if h_min <= h_max:
            h_mid = (h_min + h_max) / 2.0
        else:
            # wrap-around
            span1 = 180 - h_min
            span2 = h_max - 0
            total = span1 + span2 if (span1 + span2) > 0 else 1
            # ponto médio aproximado no wrap
            h_mid = (h_min + (total / 2.0)) % 180

        s_vals = np.linspace(s_min, s_max, samples_per_dim)
        v_vals = np.linspace(v_min, v_max, samples_per_dim)

        grid = np.zeros((samples_per_dim, samples_per_dim, 3), dtype=np.uint8)
        for i, s in enumerate(s_vals):
            for j, v in enumerate(v_vals):
                grid[i, j] = [int(h_mid), int(s), int(v)]
        rgb = cv2.cvtColor(grid, cv2.COLOR_HSV2RGB)
        ax.imshow(rgb, aspect='equal')
        ax.set_title(title)
        ax.set_xlabel('Value (brilho)')
        ax.set_ylabel('Saturation')
        ax.text(0.02, 0.98,
                f"H:{h_min}-{h_max}\nS:{s_min}-{s_max}\nV:{v_min}-{v_max}",
                transform=ax.transAxes, va='top',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        ax.set_xticks([]); ax.set_yticks([])

    def save_hsv_grid(self, output_dir: str):
        """Salva uma grade com as amostras HSV de cada forma."""
        os.makedirs(output_dir, exist_ok=True)
        names = sorted(self.shapes.keys())
        if not names:
            print("[AVISO] Nenhuma forma carregada para plotar.")
            return

        cols = 3
        rows = int(np.ceil(len(names) / cols))
        fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4 * rows))
        axes = np.atleast_2d(axes)

        for idx, name in enumerate(names):
            r, c = divmod(idx, cols)
            self._plot_color_tile(axes[r, c], name, self.shapes[name])

        # Desliga eixos vazios
        for idx in range(len(names), rows * cols):
            r, c = divmod(idx, cols)
            axes[r, c].axis('off')

        fig.suptitle('Fase3 — Faixas HSV por Forma', fontsize=16, fontweight='bold')
        plt.tight_layout()
        out = os.path.join(output_dir, 'fase3_hsv_ranges.png')
        fig.savefig(out, dpi=300, bbox_inches='tight')
        plt.close(fig)
        print(f"[OK] HSV grid salvo em: {out}")

    def save_params_txt(self, output_dir: str):
        os.makedirs(output_dir, exist_ok=True)
        out = os.path.join(output_dir, 'fase3_color_detector_parameters.txt')
        with open(out, 'w') as f:
            f.write("Fase3 Color Detector — Parâmetros carregados\n")
            f.write("=" * 48 + "\n\n")
            for name in sorted(self.shapes.keys()):
                p = self.shapes[name]
                f.write(f"[{name}]\n")
                f.write(f"  H: {p['hue_lower']}..{p['hue_upper']}\n")
                f.write(f"  S: {p['saturation_lower']}..{p['saturation_upper']}\n")
                f.write(f"  V: {p['value_lower']}..{p['value_upper']}\n\n")

            f.write("Restrições de detecção:\n")
            f.write(f"  Min Pad Area: {self.general['min_pad_area']}\n")
            f.write(f"  Max Pad Area: {self.general['max_pad_area']}\n")
            f.write(f"  Min Aspect Ratio: {self.general['min_aspect_ratio']}\n")
            f.write(f"  Max Aspect Ratio: {self.general['max_aspect_ratio']}\n")
            f.write(f"  Morph Kernel: {self.general['morph_kernel_size']}\n")
            f.write(f"  Morph Iterations: {self.general['morph_iterations']}\n")
        print(f"[OK] Parâmetros salvos em: {out}")

    def create_visualization(self, image_bgr: np.ndarray, max_mask_tiles: int = 9) -> np.ndarray:
        """
        Monta um painel com:
        [Original | Detecções] na primeira linha
        + até 'max_mask_tiles' máscaras de formas nas linhas seguintes.
        """
        candidates, debug, masks = self.find_candidates(image_bgr)

        # Tamanho base de cada tile
        tile_w, tile_h = 400, 400
        orig = cv2.resize(image_bgr, (tile_w, tile_h))
        dbg = cv2.resize(debug, (tile_w, tile_h))

        # Prepara mosaico de máscaras
        # Converte máscara (1 canal) em BGR e etiqueta
        def prep_mask_tile(name: str, m: np.ndarray) -> np.ndarray:
            m_small = cv2.resize(m, (tile_w, tile_h))
            m_bgr = cv2.cvtColor(m_small, cv2.COLOR_GRAY2BGR)
            # Tinta leve no canal G para diferenciar
            m_bgr[:, :, 1] = np.maximum(m_bgr[:, :, 1], (m_small > 0).astype(np.uint8) * 160)
            cv2.putText(m_bgr, name, (10, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
            return m_bgr

        mask_names = list(sorted(masks.keys()))
        mask_tiles = [prep_mask_tile(n, masks[n]) for n in mask_names[:max_mask_tiles]]

        # Calcula quantas colunas (3) e linhas necessárias
        cols = 3
        # Primeiro linha: original + debug + (opcional) primeira máscara
        row1_imgs = [orig, dbg]
        if mask_tiles:
            row1_imgs.append(mask_tiles.pop(0))
        else:
            row1_imgs.append(np.zeros_like(orig))

        row1 = np.hstack(row1_imgs)

        # Demais máscaras em blocos de 3
        rows_imgs = [row1]
        while mask_tiles:
            row_masks = []
            for _ in range(cols):
                if mask_tiles:
                    row_masks.append(mask_tiles.pop(0))
                else:
                    row_masks.append(np.zeros_like(orig))
            rows_imgs.append(np.hstack(row_masks))

        final_viz = np.vstack(rows_imgs)

        # Rótulos
        cv2.putText(final_viz, "Original", (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(final_viz, "Detections", (tile_w + 10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

        return final_viz

    # ---------------- Orquestração ----------------

    def save_debug_outputs(self, output_dir: str = None):
        if output_dir is None:
            output_dir = os.path.join(self.script_dir, 'debug_output')
        os.makedirs(output_dir, exist_ok=True)
        self.save_hsv_grid(output_dir)
        self.save_params_txt(output_dir)


def parse_args():
    ap = argparse.ArgumentParser(description='Fase3 Color Detector — HSV Debug Tool')
    ap.add_argument('--params', type=str,
                    default='/home/ceccon/frtl_2025_ws/src/cbr_2025/fase1/launch/params.yaml',
                    help='Caminho para o YAML de parâmetros')
    ap.add_argument('--image', type=str, help='Imagem de teste (opcional)')
    ap.add_argument('--output', type=str, help='Diretório de saída (opcional)')
    ap.add_argument('--min_ar', type=float, default=0.7, help='Razão de aspecto mínima')
    ap.add_argument('--max_ar', type=float, default=1.3, help='Razão de aspecto máxima')
    ap.add_argument('--max_area', type=int, default=120000, help='Área máxima (bbox)')
    return ap.parse_args()


def main():
    args = parse_args()

    dbg = Fase3ColorDetectorDebug(
        params_file=args.params,
        min_ar=args.min_ar,
        max_ar=args.max_ar,
        max_area=args.max_area
    )

    # Salva visuais/texto a partir só dos parâmetros
    dbg.save_debug_outputs(args.output)

    # Se tiver imagem, roda detecção e monta painel
    if args.image and os.path.exists(args.image):
        img = cv2.imread(args.image)
        if img is None:
            print(f"[ERRO] Não foi possível carregar a imagem: {args.image}")
            return
        viz = dbg.create_visualization(img, max_mask_tiles=9)
        out_dir = args.output if args.output else os.path.join(os.path.dirname(args.image), 'debug_output')
        os.makedirs(out_dir, exist_ok=True)
        out_png = os.path.join(out_dir, 'fase3_detection_debug.png')
        cv2.imwrite(out_png, viz)
        print(f"[OK] Visual de detecção salvo em: {out_png}")
        # Janela interativa (opcional)
        cv2.imshow('Fase3 Detection Debug', viz)
        print("Pressione qualquer tecla para fechar...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("[INFO] Nenhuma imagem fornecida; salvos apenas HSV grid e parâmetros.")


if __name__ == '__main__':
    main()
