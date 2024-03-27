from djitellopy import Tello

tello = Tello()

tello.connect()


battery = tello.get_battery()
print(f"Tello Battery : {battery}")


