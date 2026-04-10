import carla
import time


def format_value(value):
    """將 CARLA 的複雜物件轉為易讀的字串格式"""
    if isinstance(value, carla.Vector3D) or isinstance(value, carla.Location):
        return f"X={value.x:7.2f}, Y={value.y:7.2f}, Z={value.z:7.2f}"
    if isinstance(value, carla.Rotation):
        return f"P={value.pitch:7.2f}, Y={value.yaw:7.2f}, R={value.roll:7.2f}"
    if isinstance(value, list):
        return f"[List of {len(value)} items]"
    return str(value)


def dump_all_vehicle_params(model_id="vehicle.tesla.model3"):
    # 1. 連接至 CARLA 伺服器
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        bp_library = world.get_blueprint_library()

        # 尋找指定車輛的藍圖
        bp = bp_library.find(model_id)

        # 2. 在地圖上隨機找一個點生成車輛，以便讀取實時物理參數
        spawn_point = world.get_map().get_spawn_points()[0]
        vehicle = world.spawn_actor(bp, spawn_point)

        # 稍微等一下讓物理引擎初始化
        time.sleep(0.1)

        print(f"\n" + "=" * 60)
        print(f"  CARLA 車輛全參數報告: {model_id}")
        print("=" * 60 + "\n")

        # --- 第一部分：藍圖屬性 (Blueprint Attributes) ---
        # 這些是靜態定義，如顏色、輪胎數、角色名稱等
        print(f"[1. Blueprint Attributes]")
        for attr in bp:
            # 修正處：使用 str(attr) 或內建轉換方法
            print(f"  - {attr.id:25}: {str(attr):20} (Type: {attr.type})")

        # --- 第二部分：核心物理控制 (Vehicle Physics Control) ---
        # 這包含質量、阻力、引擎扭力曲線等
        print(f"\n[2. Vehicle Physics Control]")
        physics = vehicle.get_physics_control()

        # 動態獲取所有非隱藏屬性
        for field in dir(physics):
            if not field.startswith("_") and field != "wheels":
                val = getattr(physics, field)
                print(f"  - {field:25}: {format_value(val)}")

        # --- 第三部分：輪胎物理細節 (Wheels Physics) ---
        print(f"\n[3. Wheels Physics Details]")
        for i, wheel in enumerate(physics.wheels):
            print(f"  * Wheel {i} ({'Front' if i < 2 else 'Rear'}):")
            for w_field in dir(wheel):
                if not w_field.startswith("_"):
                    w_val = getattr(wheel, w_field)
                    print(f"    - {w_field:21}: {format_value(w_val)}")

        # --- 第四部分：幾何參數 (Geometry) ---
        print(f"\n[4. Vehicle Geometry]")
        bb = vehicle.bounding_box
        print(f"  - Bounding Box Extent   : {format_value(bb.extent)}")
        print(f"  - Bounding Box Location : {format_value(bb.location)}")

    except Exception as e:
        print(f"\n[ERROR] 執行過程中發生錯誤: {e}")

    finally:
        # 務必清理生成的車輛，避免留在伺服器中
        if "vehicle" in locals():
            vehicle.destroy()
            print(f"\n" + "=" * 60)
            print(f"  已成功銷毀測試車輛並釋放資源。")
            print("=" * 60)


if __name__ == "__main__":
    # 你可以將此處更換為任何藍圖 ID，例如 'vehicle.bmw.grandtourer'
    dump_all_vehicle_params("vehicle.tesla.model3")
