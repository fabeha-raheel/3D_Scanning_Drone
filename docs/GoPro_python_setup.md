# GoPro Camera Setup with Python

## Hardware Specifications

The following has been tested with GoPro Hero 11 Black camera. According to documentation, it works for all Hero Black versions below 11 as well.

The code implemented on a PC with Ubuntu 20.04 / Linux.

## GoPro Setup

1. Switch on the GoPro camera. Swipe downwards from the top to view the menu. Go to **Preferences** > **Wireless Connections**.
2. In the **Wireless Connections** menu, ensure that **Wireless Connections** are **On**. Also set the **Wi-fi Band** to **2.4GHz** or **5GHz** depending on your PC requirements.
3. Then, install the GoPro Quik App on your phone. Create an account or Sign In to register your camera. Make sure that location services and Bluetooth are open on your phone.
4. When the phone is ready, click on **Connect Device** > **GoPro Quik App** on the GoPro camera. Wait for the devices to Pair.  
5. Now, you should be able to see a WiFi connection for your GoPro camera in your Network connections. Connect to the camera and proceed with programming.

**Tutorial for setting up GoPro WiFi connection**: https://www.youtube.com/watch?v=5iLR-acPPvA


## Accessing and controlling GoPro camera using Python

The simplest approach I found online was this [Blog](https://flaviocopes.com/use-gopro-remote-python/#:~:text=First%20install%20the%20package%20using,you%20want%20with%20the%20webcam). It uses the GoPro API for Python available [here](https://github.com/konradit/gopro-py-api).

Install the library on your PC:
```bash
pip install goprocam
```
Import ```GoProCamera``` and ```constants``` from the library:
```python
from goprocam import GoProCamera, constants
```

Then call the ```GoProCamera.GoPro()``` method to get a camera object:
```python
gopro = GoProCamera.GoPro(constants.gpcontrol)
```

Get an overview of the camera status:
```python
gopro.overview()
```

Take a photo immediately:
```python
gopro.take_photo()
```

Record a 10-seconds video:
```python
gopro.shoot_video(10)
```

You can download the last picture or video taken and give it a file name
```python
gopro.downloadLastMedia("pic.JPG")
```
More examples can be found [here](https://github.com/KonradIT/gopro-py-api/tree/master/examples). 

Complete documentation is available [here](https://github.com/KonradIT/gopro-py-api/blob/master/docs/docs.md).

### The official GoPro Python SDK and API

The official go pro python SDK can be found at:
https://gopro.github.io/OpenGoPro/python_sdk/

The Open GoPro API is also available:
https://gopro.github.io/OpenGoPro/
