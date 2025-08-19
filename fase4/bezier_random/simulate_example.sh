#!/bin/bash

# 🚁 simulate.sh - Exemplo com Integração de Caminhos Aleatórios
# Este é um exemplo de como integrar o sistema de caminhos ao seu script existente

# Suas configurações originais...
echo "🌍 Configurando simulação Itajubá Fase 4..."

# Verificar se estamos no diretório correto
if [ ! -f "Makefile" ] || [ ! -d "src" ]; then
    echo "❌ Execute este script no diretório raiz do PX4-Autopilot"
    exit 1
fi

# === SEÇÃO DE CAMINHO ALEATÓRIO (OPCIONAL - REMOVA SE NÃO QUISER) ===
echo ""
echo "🛣️  === GERANDO CAMINHO DE TESTE ==="

# Verificar se o gerador existe
if [ -f "Tools/simulation/gz/scripts/generate_random_path.py" ]; then

    # Perguntar ao usuário se quer gerar caminho
    echo "Deseja gerar um caminho aleatório? (y/n) [y]: "
    read -r generate_path
    generate_path=${generate_path:-y}

    if [[ "$generate_path" =~ ^[Yy]$ ]]; then
        echo "Escolha o tipo de caminho:"
        echo "1) 🌀 Espiral suave (recomendado)"
        echo "2) ⚡ Zigue-zague desafiador"
        echo "3) 🔄 Retorno ao ponto inicial"
        echo "4) 🎲 Completamente aleatório"
        echo -n "Digite sua escolha [1-4] [1]: "
        read -r path_choice
        path_choice=${path_choice:-1}

        # Configurar parâmetros baseado na escolha
        case $path_choice in
            1)
                PATH_ARGS="--direction-preference spiral_left --curve-intensity 1.2 --length 25"
                echo "Gerando caminho espiral suave..."
                ;;
            2)
                PATH_ARGS="--direction-preference zigzag --curve-intensity 2.5 --segments 10 --width 0.2"
                echo "Gerando caminho zigue-zague..."
                ;;
            3)
                PATH_ARGS="--direction-preference return --length 30 --segments 6"
                echo "Gerando caminho de retorno..."
                ;;
            4)
                PATH_ARGS="--direction-preference random --curve-intensity 2.0"
                echo "Gerando caminho aleatório..."
                ;;
            *)
                PATH_ARGS="--direction-preference spiral_left"
                echo "Usando padrão (espiral)..."
                ;;
        esac

        # Gerar o caminho
        if python3 Tools/simulation/gz/scripts/generate_random_path.py \
            --output Tools/simulation/gz/worlds/random_path.sdf \
            $PATH_ARGS; then

            echo "✅ Caminho gerado com sucesso!"

            # Fazer backup do mundo original se necessário
            WORLD_FILE="Tools/simulation/gz/worlds/itajuba_fase4.sdf"
            BACKUP_FILE="Tools/simulation/gz/worlds/itajuba_fase4_backup.sdf"

            if [ ! -f "$BACKUP_FILE" ]; then
                cp "$WORLD_FILE" "$BACKUP_FILE"
                echo "📁 Backup do mundo criado"
            fi

            # Restaurar mundo original e adicionar caminho
            cp "$BACKUP_FILE" "$WORLD_FILE"

            # Integrar caminho no mundo
            if ! grep -q "random_path.sdf" "$WORLD_FILE"; then
                sed -i '/<\/world>/i\    <include><uri>random_path.sdf</uri></include>' "$WORLD_FILE"
                echo "🔗 Caminho integrado ao mundo Gazebo"
            else
                echo "🔗 Caminho já estava integrado"
            fi

        else
            echo "❌ Erro na geração do caminho, continuando sem ele..."
        fi
    else
        echo "⏭️  Pulando geração de caminho"
    fi
else
    echo "⚠️  Gerador de caminhos não encontrado"
    echo "   Execute: git checkout -- Tools/simulation/gz/scripts/"
fi

echo "🛣️  === FIM DA SEÇÃO DE CAMINHO ==="
echo ""

# === FIM DA SEÇÃO DE CAMINHO ALEATÓRIO ===

# Suas configurações de simulação originais...
echo "🚁 Preparando simulação PX4..."

# Limpar processos anteriores (opcional)
echo "🧹 Limpando simulações anteriores..."
pkill -9 px4 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
rm -rf /tmp/px4* 2>/dev/null || true

# Configurações de ambiente
export PX4_SIM_MODEL=x500_depth
export PX4_GZ_WORLD=itajuba_fase4

echo "🔧 Configurações:"
echo "   Modelo: $PX4_SIM_MODEL"
echo "   Mundo: $PX4_GZ_WORLD"
echo ""

# Iniciar simulação (substitua pelos seus comandos)
echo "🚀 Iniciando simulação..."
echo "   Para conectar QGroundControl: localhost:14550"
echo "   Para parar: Ctrl+C"
echo ""

# EXEMPLO: substitua esta linha pelo seu comando de simulação
make px4_sitl gz_x500_depth

# OU se você usa um comando diferente, substitua por:
# ./your_simulation_command_here

echo ""
echo "✅ Simulação concluída!"

# === INFORMAÇÕES ÚTEIS ===
echo ""
echo "📋 === INFORMAÇÕES ÚTEIS ==="
echo "🛣️  Arquivo do caminho: Tools/simulation/gz/worlds/random_path.sdf"
echo "🌍 Arquivo do mundo: Tools/simulation/gz/worlds/itajuba_fase4.sdf"
echo "📁 Backup do mundo: Tools/simulation/gz/worlds/itajuba_fase4_backup.sdf"
echo ""
echo "🔧 Para personalizar caminhos:"
echo "   ./Tools/simulation/gz/scripts/itajuba_path_helper.sh"
echo ""
echo "📖 Documentação completa:"
echo "   Tools/simulation/gz/docs/README.md"
echo "   Tools/simulation/gz/docs/INTEGRATION_GUIDE.md"
