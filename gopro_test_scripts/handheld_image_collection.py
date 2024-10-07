from goprocam import GoProCamera, constants
import time

gopro = GoProCamera.GoPro(constants.gpcontrol)

while True:
    gopro.take_photo(1)
    time.sleep(1)