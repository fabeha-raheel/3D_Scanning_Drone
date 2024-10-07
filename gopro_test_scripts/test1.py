# Ref: https://flaviocopes.com/use-gopro-remote-python/#:~:text=First%20install%20the%20package%20using,you%20want%20with%20the%20webcam
# Ref: https://github.com/konradit/gopro-py-api

from goprocam import GoProCamera, constants

gopro = GoProCamera.GoPro(constants.gpcontrol)

gopro.overview()

gopro.take_photo()