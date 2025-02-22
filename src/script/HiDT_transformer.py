# !/usr/bin/env python3

from pathlib import Path

### HiDT ###
import cv2
import numpy as np
import time

import sys
sys.path.append(str(Path.home()) + '/catkin_ws/src/HiDT')

import torch
import PIL.Image as PILImage
from torchvision import transforms

from hidt.networks.enhancement.RRDBNet_arch import RRDBNet
from hidt.style_transformer import StyleTransformer
from hidt.utils.preprocessing import GridCrop, enhancement_preprocessing
### HiDT ###

class HiDT_transformer():
    def __init__(self):
        common_path = str(Path.home()) + '/catkin_ws/src/HiDT'
        self.config_path = common_path + '/configs/daytime.yaml'
        self.gen_weights_path = common_path + '/trained_models/generator/daytime.pt'
        self.inference_size = 256  # the network has been trained to do inference in 256px, any higher value might lead to artifacts
        self.device = 'cuda:0'
        self.image_path = common_path + '/images/daytime/content/1.jpg'
        self.styles_path = common_path + '/styles.txt'
        self.enhancer_weights = common_path + '/trained_models/enhancer/enhancer.pth'

        self.style_transformer = StyleTransformer(self.config_path,
                                                  self.gen_weights_path,
                                                  inference_size=self.inference_size,
                                                  device=self.device)

        with open(self.styles_path) as f:
            styles = f.read()
        styles = {style.split(',')[0]: torch.tensor([float(el) for el in style.split(',')[1][1:-1].split(' ')]) for style in styles.split('\n')[:-1]}

        self.style_to_transfer = styles['sunsetred']
        self.style_to_transfer = self.style_to_transfer.view(1, 1, 3, 1).to(self.device)


    def HiDT(self, img):

        # PIL 形式に変換
        image = PILImage.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

        with torch.no_grad():
            content_decomposition = self.style_transformer.get_content(image)[0]
            decoder_input = {'content': content_decomposition['content'],
                            'intermediate_outputs': content_decomposition['intermediate_outputs'],
                            'style': self.style_to_transfer}
            transferred = self.style_transformer.trainer.gen.decode(decoder_input)['images']

        output_frame = transforms.ToPILImage()((transferred[0].cpu().clamp(-1, 1) + 1.) / 2.)

        # Pillow の Image オブジェクトを NumPy 配列に変換 (OpenCV 互換の形式に)
        image_cv = np.array(output_frame)
        # Pillow は RGB 形式なので、OpenCV の BGR に変換 (必要に応じて)
        image_cv = cv2.cvtColor(image_cv, cv2.COLOR_RGB2BGR)

        # frame_width と frame_height を指定してリサイズ
        image_cv_resized = cv2.resize(image_cv, (480, 640))

        return image_cv_resized


if __name__ == '__main__':
    transformer = HiDT_transformer()
    #img = PILImage.open('/home/shuma/catkin_ws/src/HiDT/images/daytime/content/1.jpg')
    #transformer.HiDT(img)
