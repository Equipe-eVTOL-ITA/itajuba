#!/bin/bash

# Script auxiliar para geração de caminhos para itajuba_fase4
# Usado pelo simulate.sh do workspace frtl_2025_ws

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="~/PX4-Autopilot"
WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
SCRIPTS_DIR="$PX4_DIR/Tools/simulation/gz/scripts"
TEMP_PATH_FILE="$WORLDS_DIR/generated_path.sdf"

# Função para gerar caminho
generate_path() {
    local length="${1:-20.0}"
    local width="${2:-0.3}"
    local segments="${3:-5}"
    local curve_intensity="${4:-1.5}"
    local color="${5:-0 0 1 1}"
    local seed="$6"
    local initial_direction="$7"
    local direction_preference="$8"

    echo "🎯 Gerando novo caminho para itajuba_fase4..."

    # Preparar argumentos opcionais
    local args=""
    [ -n "$seed" ] && args="$args --seed $seed"
    [ -n "$initial_direction" ] && args="$args --initial-direction $initial_direction"
    [ -n "$direction_preference" ] && args="$args --direction-preference $direction_preference"

    # Expandir ~ para o caminho completo
    local expanded_scripts_dir="${SCRIPTS_DIR/#\~/$HOME}"
    local expanded_temp_file="${TEMP_PATH_FILE/#\~/$HOME}"

    # Gerar o caminho
    python3 "$expanded_scripts_dir/generate_random_path.py" \
        --length "$length" \
        --width "$width" \
        --segments "$segments" \
        --curve-intensity "$curve_intensity" \
        --color $color \
        --output "$expanded_temp_file" \
        $args

    if [ $? -eq 0 ]; then
        echo "✅ Caminho gerado com sucesso!"
        return 0
    else
        echo "❌ Erro ao gerar caminho!"
        return 1
    fi
}

# Função para prompt interativo
interactive_path_generation() {
    echo ""
    echo "🛤️  Configuração de Caminho para itajuba_fase4"
    echo "=============================================="

    # Perguntar se quer gerar novo caminho
    read -p "Gerar novo caminho aleatório? (s/N): " generate_new

    if [[ "$generate_new" =~ ^[Ss]$ ]]; then
        echo ""
        echo "Configurações do caminho (Enter para usar padrão):"

        # Comprimento
        read -p "Comprimento (padrão: 20m): " length
        length=${length:-20.0}

        # Largura
        read -p "Largura (padrão: 0.3m): " width
        width=${width:-0.3}

        # Cor
        echo "Cor do caminho:"
        echo "  1) Azul (padrão)"
        echo "  2) Vermelho"
        echo "  3) Verde"
        echo "  4) Amarelo"
        echo "  5) Personalizada"
        read -p "Escolha (1-5): " color_choice

        case $color_choice in
            2) color="1 0 0 1" ;;
            3) color="0 1 0 1" ;;
            4) color="1 1 0 1" ;;
            5)
                read -p "RGBA (ex: 1 0.5 0 1): " color
                color=${color:-"0 0 1 1"}
                ;;
            *) color="0 0 1 1" ;;
        esac

        # Direção inicial
        echo ""
        echo "Direção inicial:"
        echo "  1) Aleatória (padrão)"
        echo "  2) Norte (0°)"
        echo "  3) Leste (90°)"
        echo "  4) Sul (180°)"
        echo "  5) Oeste (270°)"
        echo "  6) Personalizada"
        read -p "Escolha (1-6): " dir_choice

        case $dir_choice in
            2) initial_direction="0" ;;
            3) initial_direction="90" ;;
            4) initial_direction="180" ;;
            5) initial_direction="270" ;;
            6)
                read -p "Direção em graus (0-360): " initial_direction
                ;;
            *) initial_direction="" ;;
        esac

        # Padrão do caminho
        echo ""
        echo "Padrão do caminho:"
        echo "  1) Aleatório (padrão)"
        echo "  2) Espiral esquerda"
        echo "  3) Espiral direita"
        echo "  4) Zigue-zague"
        echo "  5) Retorno"
        read -p "Escolha (1-5): " pattern_choice

        case $pattern_choice in
            2) direction_preference="spiral_left" ;;
            3) direction_preference="spiral_right" ;;
            4) direction_preference="zigzag" ;;
            5) direction_preference="return" ;;
            *) direction_preference="" ;;
        esac

        # Seed para reproduzir
        read -p "Seed para reproduzir (opcional): " seed

        # Gerar o caminho
        echo ""
        if generate_path "$length" "$width" "5" "1.5" "$color" "$seed" "$initial_direction" "$direction_preference"; then
            echo ""
            echo "🎉 Caminho configurado! Iniciando simulação..."
        else
            echo ""
            echo "⚠️  Erro na geração. Continuando com caminho existente..."
        fi
    else
        echo "📁 Usando caminho existente (se houver)..."
    fi

    echo ""
}

# Função para geração rápida com argumentos
quick_generation() {
    local preset="$1"

    case "$preset" in
        "simple")
            generate_path 15 0.3 3 1.0 "0 0 1 1" "" "0" ""
            ;;
        "complex")
            generate_path 25 0.4 8 2.0 "1 0 0 1" "" "" "spiral_left"
            ;;
        "zigzag")
            generate_path 20 0.35 6 1.8 "0 1 0 1" "" "90" "zigzag"
            ;;
        "return")
            generate_path 30 0.4 6 1.5 "1 1 0 1" "" "" "return"
            ;;
        *)
            echo "Presets disponíveis: simple, complex, zigzag, return"
            return 1
            ;;
    esac
}

# Função principal
main() {
    case "${1:-interactive}" in
        "interactive")
            interactive_path_generation
            ;;
        "preset")
            quick_generation "$2"
            ;;
        "generate")
            # Argumentos diretos: length width segments curve_intensity color seed initial_dir dir_preference
            generate_path "$2" "$3" "$4" "$5" "$6 $7 $8 $9" "${10}" "${11}" "${12}"
            ;;
        "help")
            echo "Uso: $0 [modo] [argumentos]"
            echo ""
            echo "Modos:"
            echo "  interactive          Modo interativo (padrão)"
            echo "  preset <nome>        Usar preset (simple, complex, zigzag, return)"
            echo "  generate <args>      Geração direta com argumentos"
            echo "  help                 Mostra esta ajuda"
            echo ""
            echo "Exemplos:"
            echo "  $0                           # Modo interativo"
            echo "  $0 preset complex            # Caminho complexo"
            echo "  $0 preset zigzag             # Caminho zigue-zague"
            ;;
        *)
            echo "Modo inválido. Use 'help' para ver opções."
            return 1
            ;;
    esac
}

# Executar função principal se chamado diretamente
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
