from models import SessionLocal, Device

db = SessionLocal()
# 清空旧数据
db.query(Device).delete()
# 添加两条示例设备
db.add_all([
    Device(name="Test_RTSP",  protocol="rtsp",  url="rtsp://example.com/stream"),
    Device(name="Test_ONVIF", protocol="onvif", url="192.168.1.100:80")
])
db.commit()
db.close()
print("已插入测试设备")
