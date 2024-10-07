# Testing Results

## Testing with Datasets and Meshroom
1. [Pix4D Eagle Statue Dataset](https://support.pix4d.com/hc/en-us/articles/360000235126) 
    - [Demo Video](https://youtu.be/RUjHu1pxIZw)

2. [Banana ODM Dataset](https://github.com/pierotofy/dataset_banana/tree/master)
    - [Demo Video](https://youtu.be/yqJpUi0F6II)

3. [Conch ODM Dataset](https://github.com/manand881/Conch/tree/master)
    - [Demo Video](https://youtu.be/SmLFgBPGxrA)

4. Custom VTOL - images acquired using Go Pro camera
    - [Meshroom Results](https://youtu.be/PjF7tZMOvKE)
    - **Observations:** A lot of noise was observed in the final model recreation after meshing and texturing. Other softwares can be used to clean up the meshes once the model file has been generated. The mesh generation and texturing processes are dependent on some parameters which can further be fine-tuned to improve the quality of the output. This process will be explored in the future. Right now, the pipeline uses only default parameters. Furthermore, output will be better if **_the camera is held a slightly oblique angle_** to minimize background clutter in the images.