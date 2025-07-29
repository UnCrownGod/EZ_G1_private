# scripts/seed_devices.py
"""
一次性脚本：清空并插入测试设备数据。
"""

from services.camera_backend.models import SessionLocal, Device, init_db

if __name__ == "__main__":
    init_db()
    db = SessionLocal()
    try:
        db.query(Device).delete()  # 清空旧数据
        db.add_all([
            Device(name="Test_RTSP", protocol="rtsp",  url="rtsp://example.com/stream"),
            Device(name="Test_ONVIF", protocol="onvif", url="192.168.1.100:80"),
        ])
        db.commit()
        print("已插入测试设备")
    finally:
        db.close()
