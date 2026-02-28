#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# Teknofest Multi-Plane Simulation
# ═══════════════════════════════════════════════════════════════
# Birden fazla sabit kanat (plane) PX4 SITL instance'ı başlatır.
# Gazebo Classic ortamında operasyon alanı koordinatlarında spawn eder.
#
# Kullanım:
#   bash teknofest_multi_plane.sh [uçak_sayısı]
#   bash teknofest_multi_plane.sh 3    → 3 uçak spawn eder
#   bash teknofest_multi_plane.sh      → Varsayılan 2 uçak
#
# Her uçak için portlar:
#   Instance i:
#     MAVLink UDP:  14560 + i
#     MAVLink TCP:  4560 + i
#     MAVROS FCU:   udp://:1454{i}@127.0.0.1:1455{7+i}
#
# MAVROS bağlantısı (ayrı terminallerde):
#   # Uçak 0:
#   ros2 run mavros mavros_node --ros-args -r __ns:=/uav0 \
#       -p fcu_url:="udp://:14540@127.0.0.1:14557"
#
#   # Uçak 1:
#   ros2 run mavros mavros_node --ros-args -r __ns:=/uav1 \
#       -p fcu_url:="udp://:14541@127.0.0.1:14558"
# ═══════════════════════════════════════════════════════════════

set -e

NUM_VEHICLES=${1:-2}
PX4_DIR=~/PX4-Autopilot
BUILD_DIR=$PX4_DIR/build/px4_sitl_default
GAZEBO_MODELS=$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic
MODEL_NAME="plane"

# ═══════════════════════════════════════════════════════════════
# Operasyon Alanı Merkezi (GPS → Gazebo spawn offset hesabı)
# Köşeler:
#   1: 40.230761, 29.001866
#   2: 40.229318, 29.009345
#   3: 40.233739, 29.009292
#   4: 40.233848, 28.998916
# ═══════════════════════════════════════════════════════════════

# Uçaklar arası spawn mesafesi (metre)
SPAWN_SPACING=10

echo "═══════════════════════════════════════════════════"
echo "  🛩  Teknofest Multi-Plane Simülasyonu"
echo "  Uçak sayısı: $NUM_VEHICLES"
echo "  Model: $MODEL_NAME (sabit kanat)"
echo "═══════════════════════════════════════════════════"

# ─── Mevcut süreçleri temizle ───
echo ""
echo "🧹 Mevcut simülasyon süreçleri temizleniyor..."
pkill -9 px4 2>/dev/null || true
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
sleep 2

# ─── PX4 Gazebo ortamını ayarla ───
echo "⚙  Gazebo ortamı ayarlanıyor..."
cd $PX4_DIR
source Tools/simulation/gazebo-classic/setup_gazebo.bash \
    $(pwd) $(pwd)/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

# ─── PX4_SIM_MODEL ayarla (plane airframe seçimi için) ───
export PX4_SIM_MODEL=$MODEL_NAME

# ─── Gazebo Server'ı başlat ───
echo ""
echo "🌍 Gazebo server başlatılıyor..."
gzserver --verbose $GAZEBO_MODELS/worlds/empty.world &
GZSERVER_PID=$!
sleep 5

# Server başladı mı kontrol et
if ! kill -0 $GZSERVER_PID 2>/dev/null; then
    echo "❌ Gazebo server başlatılamadı!"
    exit 1
fi
echo "✅ Gazebo server çalışıyor (PID: $GZSERVER_PID)"

# ─── Plane SDF'ini jinja'dan oluştur (eğer yoksa) ───
PLANE_SDF="/tmp/plane_sim.sdf"
if [ -f "$GAZEBO_MODELS/models/$MODEL_NAME/$MODEL_NAME.sdf.jinja" ]; then
    echo "📄 Plane SDF oluşturuluyor (jinja template'den)..."
    python3 $GAZEBO_MODELS/scripts/jinja_gen.py \
        $GAZEBO_MODELS/models/$MODEL_NAME/$MODEL_NAME.sdf.jinja \
        $GAZEBO_MODELS \
        --mavlink_tcp_port 4560 \
        --mavlink_udp_port 14560 \
        --serial_enabled 0 \
        --serial_device /dev/ttyACM0 \
        --serial_baudrate 921600 \
        --hil_mode 0 \
        --output-file $PLANE_SDF
elif [ -f "$GAZEBO_MODELS/models/$MODEL_NAME/$MODEL_NAME.sdf" ]; then
    PLANE_SDF="$GAZEBO_MODELS/models/$MODEL_NAME/$MODEL_NAME.sdf"
else
    echo "❌ Plane SDF modeli bulunamadı!"
    exit 1
fi

# ─── Uçakları spawn et ───
echo ""
echo "🛩  Uçaklar spawn ediliyor..."

for i in $(seq 0 $(($NUM_VEHICLES - 1))); do
    X_POS=$(($i * $SPAWN_SPACING))
    Y_POS=0
    
    MAVLINK_UDP=$((14560 + $i))
    MAVLINK_TCP=$((4560 + $i))
    
    echo ""
    echo "─── Uçak $i ───"
    echo "  Konum: X=$X_POS, Y=$Y_POS"
    echo "  MAVLink UDP: $MAVLINK_UDP, TCP: $MAVLINK_TCP"
    echo "  MAVROS FCU: udp://:$((14540 + $i))@127.0.0.1:$((14557 + $i))"
    
    # ─── PX4 SITL Instance başlat ───
    $BUILD_DIR/bin/px4 \
        -i $i \
        -d $BUILD_DIR/etc \
        -w $PX4_DIR/sitl_${MODEL_NAME}_$i \
        -s etc/init.d-posix/rcS \
        > /tmp/px4_${MODEL_NAME}_$i.log 2>&1 &
    
    PX4_PID=$!
    echo "  PX4 PID: $PX4_PID"
    
    # PX4'ün başlaması için bekle
    sleep 4
    
    # ─── Modeli Gazebo'ya spawn et ───
    # Her uçak için farklı MAVLink portları ile SDF oluştur
    PLANE_SDF_I="/tmp/plane_sim_$i.sdf"
    if [ -f "$GAZEBO_MODELS/models/$MODEL_NAME/$MODEL_NAME.sdf.jinja" ]; then
        python3 $GAZEBO_MODELS/scripts/jinja_gen.py \
            $GAZEBO_MODELS/models/$MODEL_NAME/$MODEL_NAME.sdf.jinja \
            $GAZEBO_MODELS \
            --mavlink_tcp_port $MAVLINK_TCP \
            --mavlink_udp_port $MAVLINK_UDP \
            --serial_enabled 0 \
            --serial_device /dev/ttyACM0 \
            --serial_baudrate 921600 \
            --hil_mode 0 \
            --output-file $PLANE_SDF_I
        
        gz model \
            --spawn-file=$PLANE_SDF_I \
            --model-name=${MODEL_NAME}_$i \
            -x $X_POS -y $Y_POS -z 0
    else
        gz model \
            --spawn-file=$PLANE_SDF \
            --model-name=${MODEL_NAME}_$i \
            -x $X_POS -y $Y_POS -z 0
    fi
    
    echo "  ✅ Uçak $i spawn edildi!"
done

# ─── Gazebo Client (GUI) başlat ───
echo ""
echo "🖥  Gazebo GUI başlatılıyor..."
gzclient &

# ─── Özet ───
echo ""
echo "═══════════════════════════════════════════════════"
echo "  ✅ $NUM_VEHICLES adet $MODEL_NAME spawn edildi!"
echo ""
echo "  📡 MAVROS bağlantıları (ayrı terminallerde):"
echo ""
for i in $(seq 0 $(($NUM_VEHICLES - 1))); do
    FCU_PORT=$((14540 + $i))
    REMOTE_PORT=$((14557 + $i))
    echo "  Uçak $i:"
    echo "    ros2 run mavros mavros_node --ros-args \\"
    echo "        -r __ns:=/uav$i \\"
    echo "        -p fcu_url:=\"udp://:${FCU_PORT}@127.0.0.1:${REMOTE_PORT}\""
    echo ""
done
echo "  🎯 Misyon (ayrı terminallerde):"
echo ""
for i in $(seq 0 $(($NUM_VEHICLES - 1))); do
    echo "  Uçak $i:"
    echo "    cd ~/otopilot_ws && source install/setup.bash"
    echo "    ros2 run iha_otopilot enemy_plane --ros-args -p namespace:=\"uav$i\" -p vehicle_id:=$i"
    echo ""
done
echo "═══════════════════════════════════════════════════"
echo ""
echo "  Durdurmak için: Ctrl+C"

# ─── Temizlik ───
trap "echo ''; echo '🛑 Simülasyon durduruluyor...'; pkill -9 px4; pkill -9 gz; exit 0" INT TERM
wait
