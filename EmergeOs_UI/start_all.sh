#!/bin/bash
# 一键启动 DimOS 和 EmergeUI 前端界面

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIMOS_DIR="/home/emergeos/Share_pgx/ZLP/dimos"

# 解析参数
SIMULATION=false
DIMOS_CONFIG="unitree-go2-agentic"

while [[ $# -gt 0 ]]; do
    case $1 in
        -s|--simulation)
            SIMULATION=true
            shift
            ;;
        -h|--help)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  -s, --simulation    使用仿真模式启动 DimOS"
            echo "  -h, --help          显示帮助信息"
            exit 0
            ;;
        *)
            DIMOS_CONFIG="$1"
            shift
            ;;
    esac
done

# 构建 DimOS 启动命令
if [ "$SIMULATION" = true ]; then
    DIMOS_CMD="dimos --simulation run $DIMOS_CONFIG"
    MODE_LABEL="仿真模式"
else
    DIMOS_CMD="dimos run $DIMOS_CONFIG"
    MODE_LABEL="实机模式"
fi

echo "======================================"
echo "  EmergeOS Dashboard 启动脚本"
echo "======================================"
echo "运行模式: $MODE_LABEL"
echo ""

# 函数：检查进程是否运行
check_running() {
    pgrep -f "$1" > /dev/null 2>&1
}

# 启动 DimOS（在新终端中）
echo "[1/2] 启动 DimOS..."
if check_running "dimos run"; then
    echo "  ⚠️  DimOS 已在运行，跳过启动"
else
    echo "  在新终端中启动 DimOS..."
    gnome-terminal --title="DimOS - $MODE_LABEL" -- bash -c \
        "cd $DIMOS_DIR && source .venv/bin/activate && $DIMOS_CMD; exec bash"
    sleep 2
    echo "  ✓ DimOS 已在新终端启动"
fi

# 启动前端
echo "[2/2] 启动 EmergeUI 前端..."
if check_running "next dev"; then
    echo "  ⚠️  前端已在运行，跳过启动"
else
    cd "$SCRIPT_DIR"
    nohup npm run dev > /tmp/emergeui.log 2>&1 &
    sleep 5
    if check_running "next dev"; then
        echo "  ✓ 前端启动成功"
        echo "  访问地址: http://localhost:3000"
    else
        echo "  ✗ 前端启动失败，请检查 /tmp/emergeui.log"
    fi
fi

# 显示状态
echo ""
echo "系统状态:"
echo "--------------------------------------"
echo "  DimOS:      $(check_running 'dimos run' && echo '● 运行中' || echo '○ 未运行')"
echo "  Bridge:     $(check_running 'bridge_dimos_ui' && echo '● 运行中' || echo '○ 未运行')"
echo "  Hermes:     $(check_running 'hermes_bridge' && echo '● 运行中' || echo '○ 未运行')"
echo "  Next.js:    $(check_running 'next dev' && echo '● 运行中' || echo '○ 未运行')"
echo ""
echo "  前端界面:   http://localhost:3000"
echo "--------------------------------------"
echo ""
echo "停止所有服务: npm run killall && pkill -f 'dimos run'"
