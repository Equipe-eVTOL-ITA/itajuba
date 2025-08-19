# 🛣️ Gerador de Caminhos Aleatórios - PX4 Simulation

Sistema completo para gerar caminhos azuis aleatórios em simulações do PX4-Autopilot com Gazebo Garden/Harmonic.

## 🚀 Início Rápido

### Opção 1: Interface Interativa (Mais Fácil)
```bash
cd /home/marconipavan/PX4-Autopilot
./Tools/simulation/gz/scripts/itajuba_path_helper.sh
```

### Opção 2: Comando Direto
```bash
python3 Tools/simulation/gz/scripts/generate_random_path.py \
  --direction-preference spiral_left \
  --length 25 \
  --segments 6
```

### Opção 3: Integração Automática
```bash
# Adicione estas 3 linhas ao seu simulate.sh:
echo "Gerando caminho..."
python3 Tools/simulation/gz/scripts/generate_random_path.py --output Tools/simulation/gz/worlds/random_path.sdf
sed -i '/<\/world>/i\    <include><uri>random_path.sdf</uri></include>' Tools/simulation/gz/worlds/itajuba_fase4.sdf
```

## 🎯 Recursos Principais

- **🎲 Geração Aleatória**: Caminhos únicos a cada execução
- **🧭 Controle Direcional**: 8 padrões diferentes (espirais, zigue-zague, retorno)
- **📐 Curvas de Bézier**: Caminhos suaves e naturais
- **🎨 Customização Visual**: Cores, largura e transparência ajustáveis
- **🔧 Fácil Integração**: Scripts prontos para usar
- **📱 Interface Amigável**: Helper interativo com menus

## 🗺️ Padrões de Caminho Disponíveis

| Padrão | Descrição | Ideal Para |
|--------|-----------|------------|
| `spiral_left` | Espiral anti-horário | Testes de curva suave |
| `spiral_right` | Espiral horário | Navegação circular |
| `zigzag` | Zigue-zague alternado | Testes de agilidade |
| `return` | Retorna ao início | Missões de ida e volta |
| `north` | Tendência para norte | Voos direcionais |
| `east/west/south` | Outras direções | Testes específicos |
| `random` | Completamente aleatório | Desafios máximos |

## 📋 Parâmetros Completos

```bash
python3 generate_random_path.py [OPTIONS]

Opções Principais:
  --length FLOAT              Comprimento total em metros [padrão: 20.0]
  --width FLOAT               Largura do caminho em metros [padrão: 0.3]
  --segments INT              Número de segmentos [padrão: 4]
  --curve-intensity FLOAT     Intensidade das curvas 0-3 [padrão: 1.5]
  --output PATH               Arquivo de saída [padrão: random_path.sdf]
  --seed INT                  Seed para reproduzir o mesmo caminho

Controle Direcional:
  --initial-direction FLOAT   Direção inicial em graus (0=Norte, 90=Leste, 180=Sul, 270=Oeste)
  --direction-preference      Padrão direcional: {spiral_left, spiral_right, zigzag, return, north, south, east, west, random}

Aparência:
  --color R G B A            Cor RGBA [padrão: 0 0 1 1] (azul opaco)
```

## 🎨 Exemplos de Uso

### Caminho Espiral Suave
```bash
python3 generate_random_path.py \
  --direction-preference spiral_left \
  --length 30 \
  --curve-intensity 1.0 \
  --color 0 1 0 1  # Verde
```

### Percurso de Zigue-zague Intenso
```bash
python3 generate_random_path.py \
  --direction-preference zigzag \
  --segments 12 \
  --curve-intensity 3.0 \
  --width 0.2 \
  --color 1 0 0 1  # Vermelho
```

### Missão de Retorno ao Base
```bash
python3 generate_random_path.py \
  --direction-preference return \
  --initial-direction 45 \
  --length 25 \
  --segments 6 \
  --color 1 1 0 0.8  # Amarelo semi-transparente
```

### Caminho Reproduzível para Testes
```bash
python3 generate_random_path.py \
  --seed 12345 \
  --direction-preference spiral_right \
  --length 20
# Sempre gera o mesmo caminho
```

## 🔧 Estrutura do Sistema

```
PX4-Autopilot/Tools/simulation/gz/
├── scripts/
│   ├── generate_random_path.py      # 🐍 Gerador principal Python
│   ├── itajuba_path_helper.sh       # 🖥️  Interface interativa
│   └── px4_simulation_with_path.sh  # 🚁 Launcher integrado
├── worlds/
│   ├── itajuba_fase4.sdf           # 🌍 Mundo base
│   └── random_path.sdf             # 🛣️  Caminho gerado
└── docs/
    ├── README.md                   # 📖 Esta documentação
    ├── INTEGRATION_GUIDE.md        # 📋 Guia de integração
    └── QUICK_INTEGRATION.md        # ⚡ Integração rápida
```

## 🤖 Como Funciona

1. **Análise Matemática**: Calcula pontos de controle usando curvas de Bézier cúbicas
2. **Geração de Segmentos**: Divide o caminho em seções suaves e conectadas
3. **Controle Direcional**: Aplica preferências direcionais para padrões específicos
4. **Renderização SDF**: Converte pontos matemáticos em geometria 3D Gazebo
5. **Integração Automática**: Inclui o caminho no mundo de simulação

## 📊 Algoritmo de Direção

```python
# Pseudocódigo simplificado
def calculate_direction_change(current_direction, curve_intensity, preference):
    if preference == 'spiral_left':
        return current_direction + curve_intensity * 0.5
    elif preference == 'spiral_right':
        return current_direction - curve_intensity * 0.5
    elif preference == 'zigzag':
        return current_direction + curve_intensity * (alternating_sign)
    elif preference == 'return':
        target = current_direction + π  # Oposto
        return lerp(current_direction, target, 0.3)
    else:
        return current_direction + random(-curve_intensity, curve_intensity)
```

## ⚙️ Configuração do Ambiente

### Pré-requisitos
```bash
# Python 3 com NumPy
sudo apt install python3 python3-pip
pip3 install numpy

# PX4-Autopilot configurado
# Gazebo Garden ou Harmonic instalado
```

### Verificação da Instalação
```bash
# Verificar Python
python3 --version
python3 -c "import numpy; print('NumPy OK')"

# Verificar arquivos
ls Tools/simulation/gz/scripts/generate_random_path.py
ls Tools/simulation/gz/scripts/itajuba_path_helper.sh

# Teste básico
python3 Tools/simulation/gz/scripts/generate_random_path.py --help
```

## 🎮 Interface Interativa

O script `itajuba_path_helper.sh` oferece um menu interativo:

```
🛣️  === GERADOR DE CAMINHOS ALEATÓRIOS ===

Escolha uma opção:
1) 🎲 Gerar caminho aleatório básico
2) 🌀 Gerar espiral (esquerda/direita)
3) ⚡ Gerar zigue-zague
4) 🔄 Gerar caminho de retorno
5) 🎯 Configuração personalizada
6) 🚀 Geração rápida (presets)
7) 📖 Mostrar ajuda
8) 🚪 Sair

Digite sua escolha [1-8]:
```

## 🔍 Solução de Problemas

### Erro: "ModuleNotFoundError: No module named 'numpy'"
```bash
pip3 install numpy
# ou para sistema:
sudo apt install python3-numpy
```

### Erro: "Permission denied"
```bash
chmod +x Tools/simulation/gz/scripts/itajuba_path_helper.sh
chmod +x Tools/simulation/gz/scripts/generate_random_path.py
```

### Caminho não aparece na simulação
```bash
# Verificar geração
ls -la Tools/simulation/gz/worlds/random_path.sdf

# Verificar inclusão no mundo
grep "random_path" Tools/simulation/gz/worlds/itajuba_fase4.sdf

# Reintegrar manualmente
sed -i '/<\/world>/i\    <include><uri>random_path.sdf</uri></include>' Tools/simulation/gz/worlds/itajuba_fase4.sdf
```

### Curvas muito acentuadas
```bash
# Reduzir intensidade
python3 generate_random_path.py --curve-intensity 0.5

# Aumentar número de segmentos
python3 generate_random_path.py --segments 8
```

## 📈 Casos de Uso

### Para Desenvolvimento de Algoritmos
- **Testes de Navegação**: Caminhos variados para validar algoritmos
- **Detecção de Obstáculos**: Referências visuais para sensores
- **Planejamento de Trajetória**: Comparação com rotas planejadas

### Para Demonstrações
- **Apresentações**: Caminhos visualmente atraentes
- **Competições**: Percursos padronizados mas únicos
- **Treinamento**: Cenários variados para pilotos

### Para Testes Automatizados
- **CI/CD**: Caminhos reproduzíveis com `--seed`
- **Benchmarks**: Métricas de performance consistentes
- **Validação**: Testes de regressão automáticos

## 🚀 Extensões Futuras

- [ ] **Obstáculos Dinâmicos**: Adicionar objetos móveis ao caminho
- [ ] **Altitudes Variadas**: Caminhos 3D com mudanças de altura
- [ ] **Waypoints GPS**: Conversão para coordenadas reais
- [ ] **Validação de Segurança**: Verificação de colisões
- [ ] **Interface Gráfica**: GUI para configuração visual
- [ ] **Exportação KML**: Para visualização no Google Earth

## 📞 Suporte e Contribuições

### Estrutura de Arquivos Principal
- **`generate_random_path.py`**: Motor principal (Python 3 + NumPy)
- **`itajuba_path_helper.sh`**: Interface de usuário (Bash)
- **`INTEGRATION_GUIDE.md`**: Guia detalhado de integração

### Personalização
O sistema é altamente modular. Para adicionar novos padrões:

1. Edite a função `calculate_direction_change()` em `generate_random_path.py`
2. Adicione nova opção no argparse `--direction-preference`
3. Inclua o padrão no helper interativo

---

**🎯 Dica Rápida**: Para uso imediato, execute `./Tools/simulation/gz/scripts/itajuba_path_helper.sh` e escolha a opção 6 (Geração rápida)!
