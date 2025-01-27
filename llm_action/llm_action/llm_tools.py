from llama_index.core.img_utils import img_2_b64
from PIL import Image as PILImage
from llama_index.core.schema import ImageDocument
from cv_bridge import CvBridge
import cv2
import base64
from sensor_msgs.msg import Image
from typing import List
    
class LamaIndexToRosTools():
    def __init__(self) -> None:
        self.bridge = CvBridge()
        
    def ros2_to_image_document(self, input_image:Image) -> List[ImageDocument]:  
        opencv_img = self.bridge.imgmsg_to_cv2(input_image, "bgr8")
        # Convert from BGR (OpenCV) to RGB (PIL) color space
        rgb_image = cv2.cvtColor(opencv_img, cv2.COLOR_BGR2RGB)

        # Convert to PIL Image
        pil_image = PILImage.fromarray(rgb_image)
        b64_img = img_2_b64(pil_image)
        image_nodes = [ImageDocument(image=b64_img, mimetype="jpeg")]
        return image_nodes
    
