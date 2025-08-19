#!/bin/bash# 🚁 PX4 Simulation Launcher com Caminhos Aleatórios# Integra geração de caminhos com a simulação PX4# Uso: ./px4_simulation_with_path.sh [opções]set -e# Configurações padrãoDEFAULT_VEHICLE="x500_depth"DEFAULT_WORLD="itajuba_fase4"DEFAULT_PATH_PREFERENCE="spiral_left"DEFAULT_PATH_LENGTH="25"DEFAULT_PATH_SEGMENTS="6"DEFAULT_CURVE_INTENSITY="1.5"# Cores para outputRED='\033[0;31m'GREEN='\033[0;32m'BLUE='\033[0;34m'YELLOW='\033[1;33m'NC='\033[0m' # No Color# Função para mostrar helpshow_help() {    echo "🚁 PX4 Simulation Launcher com Caminhos Aleatórios"    echo ""    echo "Uso: $0 [OPÇÕES]"    echo ""    echo "Opções de Simulação:"    echo "  -v, --vehicle VEHICLE    Veículo PX4 [padrão: $DEFAULT_VEHICLE]"    echo "  -w, --world WORLD        Mundo Gazebo [padrão: $DEFAULT_WORLD]"    echo "  --no-path               Não gerar caminho aleatório"    echo "  --clean                 Limpar simulações anteriores"    echo ""    echo "Opções de Caminho:"    echo "  -p, --path-preference   Padrão do caminho:"    echo "                          spiral_left, spiral_right, zigzag, return"    echo "                          north, south, east, west, random"    echo "                          [padrão: $DEFAULT_PATH_PREFERENCE]"    echo "  -l, --length LENGTH     Comprimento do caminho [padrão: $DEFAULT_PATH_LENGTH]"    echo "  -s, --segments SEGMENTS Número de segmentos [padrão: $DEFAULT_PATH_SEGMENTS]"    echo "  -c, --curve-intensity   Intensidade das curvas [padrão: $DEFAULT_CURVE_INTENSITY]"    echo "  --path-width WIDTH      Largura do caminho [padrão: 0.3]"    echo "  --path-color R G B A    Cor RGBA [padrão: 0 0 1 1]"    echo "  --seed SEED             Seed para reproduzir o caminho"    echo ""    echo "Opções Gerais:"    echo "  -h, --help              Mostrar esta ajuda"    echo "  --verbose               Output detalhado"    echo ""    echo "Exemplos:"    echo "  $0                                    # Simulação padrão com caminho espiral"    echo "  $0 --path-preference zigzag          # Caminho zigue-zague"    echo "  $0 --no-path                         # Sem caminho aleatório"    echo "  $0 --length 40 --segments 10         # Caminho longo e complexo"    echo "  $0 --seed 12345                      # Caminho reproduzível"}# Função para logginglog_info() {    echo -e "${BLUE}[INFO]${NC} $1"}log_success() {    echo -e "${GREEN}[SUCCESS]${NC} $1"}log_warning() {    echo -e "${YELLOW}[WARNING]${NC} $1"}log_error() {    echo -e "${RED}[ERROR]${NC} $1"}# Verificar se estamos no diretório corretocheck_px4_directory() {    if [ ! -f "Makefile" ] || [ ! -d "src" ] || [ ! -f "package.xml" ]; then        log_error "Este script deve ser executado no diretório raiz do PX4-Autopilot"        log_info "Navegue para o diretório correto: cd /path/to/PX4-Autopilot"        exit 1    fi}# Função para limpar simulações anteriorescleanup_simulation() {    log_info "Limpando simulações anteriores..."        # Matar processos PX4 e Gazebo    pkill -9 px4 2>/dev/null || true    pkill -9 -f "gz sim" 2>/dev/null || true    pkill -9 gzserver 2>/dev/null || true        # Limpar arquivos temporários    rm -rf /tmp/px4* 2>/dev/null || true        log_success "Limpeza concluída"}# Função para gerar caminho aleatóriogenerate_random_path() {    local path_preference="$1"    local length="$2"    local segments="$3"    local curve_intensity="$4"    local width="$5"    local color="$6"    local seed="$7"    local world="$8"        log_info "Gerando caminho aleatório..."        # Verificar se o script Python existe    local python_script="Tools/simulation/gz/scripts/generate_random_path.py"    if [ ! -f "$python_script" ]; then        log_error "Script Python não encontrado: $python_script"        return 1    fi        # Verificar dependências Python    if ! python3 -c "import numpy" 2>/dev/null; then        log_error "NumPy não instalado. Execute: pip3 install numpy"        return 1    fi        # Preparar argumentos    local output_path="Tools/simulation/gz/worlds/random_path.sdf"    local cmd_args=(        "--output" "$output_path"        "--length" "$length"        "--segments" "$segments"        "--curve-intensity" "$curve_intensity"        "--width" "$width"    )        # Adicionar preferência direcional se especificada    if [ -n "$path_preference" ] && [ "$path_preference" != "none" ]; then        cmd_args+=("--direction-preference" "$path_preference")    fi        # Adicionar cor se especificada    if [ -n "$color" ]; then        cmd_args+=("--color" $color)    fi        # Adicionar seed se especificado    if [ -n "$seed" ]; then        cmd_args+=("--seed" "$seed")    fi        # Executar geração    if python3 "$python_script" "${cmd_args[@]}"; then        log_success "Caminho gerado: $output_path"                # Integrar caminho no mundo
        integrate_path_into_world "$world" "$output_path"
        return 0
    else
        log_error "Falha na geração do caminho"
        return 1
    fi
}

# Função para integrar caminho no mundo Gazebo
integrate_path_into_world() {
    local world="$1"
    local path_file="$2"
    local world_path="Tools/simulation/gz/worlds/${world}.sdf"
    local backup_path="Tools/simulation/gz/worlds/${world}_backup.sdf"

    # Verificar se o mundo existe
    if [ ! -f "$world_path" ]; then
        log_error "Arquivo do mundo não encontrado: $world_path"
        return 1
    fi

    # Fazer backup se necessário
    if [ ! -f "$backup_path" ]; then
        cp "$world_path" "$backup_path"
        log_info "Backup do mundo criado: $backup_path"
    fi

    # Restaurar do backup
    cp "$backup_path" "$world_path"

    # Adicionar include do caminho
    if grep -q "random_path.sdf" "$world_path"; then
        log_info "Caminho já incluído no mundo"
    else
        # Encontrar a tag </world> e adicionar include antes dela
        sed -i '/<\/world>/i\    <include><uri>random_path.sdf</uri></include>' "$world_path"
        log_success "Caminho integrado ao mundo: $world"
    fi
}

# Função principal de simulação
start_px4_simulation() {
    local vehicle="$1"
    local world="$2"

    log_info "Iniciando simulação PX4..."
    log_info "Veículo: $vehicle"
    log_info "Mundo: $world"

    # Comando de simulação PX4
    local sim_target="px4_sitl_default"
    local gazebo_target="gz_${vehicle}"

    log_info "Compilando PX4..."
    if ! make "$sim_target" "$gazebo_target"; then
        log_error "Falha na compilação do PX4"
        return 1
    fi

    # Configurar variáveis de ambiente
    export PX4_SIM_MODEL="$vehicle"
    export PX4_GZ_WORLD="$world"

    log_success "Simulação iniciada com sucesso!"
    log_info "Para conectar com QGroundControl, use porta 14550"
    log_info "Para parar a simulação, pressione Ctrl+C"
}

# Função principal
main() {
    local vehicle="$DEFAULT_VEHICLE"
    local world="$DEFAULT_WORLD"
    local path_preference="$DEFAULT_PATH_PREFERENCE"
    local length="$DEFAULT_PATH_LENGTH"
    local segments="$DEFAULT_PATH_SEGMENTS"
    local curve_intensity="$DEFAULT_CURVE_INTENSITY"
    local width="0.3"
    local color="0 0 1 1"
    local seed=""
    local generate_path=true
    local do_cleanup=false
    local verbose=false

    # Parse argumentos
    while [[ $# -gt 0 ]]; do
        case $1 in
            -v|--vehicle)
                vehicle="$2"
                shift 2
                ;;
            -w|--world)
                world="$2"
                shift 2
                ;;
            -p|--path-preference)
                path_preference="$2"
                shift 2
                ;;
            -l|--length)
                length="$2"
                shift 2
                ;;
            -s|--segments)
                segments="$2"
                shift 2
                ;;
            -c|--curve-intensity)
                curve_intensity="$2"
                shift 2
                ;;
            --path-width)
                width="$2"
                shift 2
                ;;
            --path-color)
                color="$2 $3 $4 $5"
                shift 5
                ;;
            --seed)
                seed="$2"
                shift 2
                ;;
            --no-path)
                generate_path=false
                shift
                ;;
            --clean)
                do_cleanup=true
                shift
                ;;
            --verbose)
                verbose=true
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                log_error "Opção desconhecida: $1"
                show_help
                exit 1
                ;;
        esac
    done

    # Mostrar configuração se verbose
    if [ "$verbose" = true ]; then
        log_info "Configuração:"
        echo "  Veículo: $vehicle"
        echo "  Mundo: $world"
        echo "  Gerar caminho: $generate_path"
        if [ "$generate_path" = true ]; then
            echo "  Preferência: $path_preference"
            echo "  Comprimento: $length"
            echo "  Segmentos: $segments"
            echo "  Intensidade: $curve_intensity"
            echo "  Largura: $width"
            echo "  Cor: $color"
            [ -n "$seed" ] && echo "  Seed: $seed"
        fi
    fi

    # Verificar diretório
    check_px4_directory

    # Limpeza se solicitada
    if [ "$do_cleanup" = true ]; then
        cleanup_simulation
    fi

    # Gerar caminho se solicitado
    if [ "$generate_path" = true ]; then
        if ! generate_random_path "$path_preference" "$length" "$segments" "$curve_intensity" "$width" "$color" "$seed" "$world"; then
            log_warning "Falha na geração do caminho, continuando sem ele..."
        fi
    fi

    # Iniciar simulação
    start_px4_simulation "$vehicle" "$world"
}

# Executar função principal
main "$@"
