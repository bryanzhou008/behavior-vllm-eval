#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as tmodels
import math
from igibson.agents.savi_rt.models.Unet_parts import UNetUp


# from ss_baselines.common.utils import Flatten
from utils.utils import Flatten, d3_40_colors_rgb


class VisualCNN(nn.Module):
    # unrelated
    r"""A Simple 3-Conv CNN followed by a fully connected layer

    Takes in observations and produces an embedding of the rgb and/or depth components

    Args:
        observation_space: The observation_space of the agent
        output_size: The size of the embedding vector
    """

    def __init__(self, observation_space, output_size, extra_rgb=False):
        super().__init__()
        self._output_size = output_size
        if "rgb" in observation_space.spaces and not extra_rgb:
            self._n_input_rgb = observation_space.spaces["rgb"].shape[2]
        else:
            self._n_input_rgb = 0

        if "depth" in observation_space.spaces:
            self._n_input_depth = observation_space.spaces["depth"].shape[2]
        else:
            self._n_input_depth = 0
            
        if "floorplan_map" in observation_space.spaces:
            self._n_input_map = 1
        else:
            self._n_input_map = 0
       
        # kernel size for different CNN layers
        self._cnn_layers_kernel_size = [(7, 7), (4, 4), (3, 3)]

        # strides for different CNN layers
        self._cnn_layers_stride = [(2, 2), (2, 2), (2, 2)]

        # paddings for different CNN layers
        self._cnn_layers_padding = [(3, 3), (2, 2), (2, 2)]

        self.resnet = tmodels.resnet18(pretrained=True)

        self.n_channels_out = 128
        
        self.out_scale = np.array([16, 16])

        max_out_scale = np.amax(self.out_scale)
        n_upscale = int(np.ceil(math.log(max_out_scale, 2)))
        self.scaler = nn.ModuleList([
            UNetUp(max(self.n_channels_out // (2**i), 64),
                   max(self.n_channels_out // (2**(i + 1)), 64),
                   bilinear=False,
                   norm='batchnorm') for i in range(n_upscale)
        ])

        if self._n_input_rgb > 0:
            cnn_dims = np.array(
                observation_space.spaces["rgb"].shape[:2], dtype=np.float32
            )          
        elif self._n_input_depth > 0:
            cnn_dims = np.array(
                observation_space.spaces["depth"].shape[:2], dtype=np.float32
            )
            
        if self.is_blind:
            self.cnn = nn.Sequential()
        else:
            self._input_shape = (self._n_input_rgb + self._n_input_depth + self._n_input_map,
                                 int(cnn_dims[0]), int(cnn_dims[1]))
            for kernel_size, stride in zip(
                self._cnn_layers_kernel_size, self._cnn_layers_stride
            ):
                cnn_dims = self._conv_output_dim(
                    dimension=cnn_dims,
                    padding=np.array([0, 0], dtype=np.float32),
                    dilation=np.array([1, 1], dtype=np.float32),
                    kernel_size=np.array(kernel_size, dtype=np.float32),
                    stride=np.array(stride, dtype=np.float32),
                )

            self.cnn = nn.Sequential(
                nn.Conv2d(
                    in_channels=self._n_input_rgb + self._n_input_depth + self._n_input_map,
                    out_channels=64,
                    kernel_size=self._cnn_layers_kernel_size[0],
                    stride=self._cnn_layers_stride[0],
                    padding=self._cnn_layers_padding[0],
                ),
                self.resnet.bn1,
                self.resnet.relu,
                self.resnet.maxpool,
                self.resnet.layer1,
                self.resnet.layer2, #(512, H/8, W/8)
                
                # nn.Conv2d(
                #     in_channels=32,
                #     out_channels=64,
                #     kernel_size=self._cnn_layers_kernel_size[1],
                #     stride=self._cnn_layers_stride[1],
                # ),
                # nn.ReLU(True),
                # nn.Conv2d(
                #     in_channels=64,
                #     out_channels=64,
                #     kernel_size=self._cnn_layers_kernel_size[2],
                #     stride=self._cnn_layers_stride[2],
                # ),
                # #  nn.ReLU(True),
                # Flatten(),
                # nn.Linear(64 * cnn_dims[0] * cnn_dims[1], output_size),
                # nn.ReLU(True),
            )

        # self.layer_init()

    def _conv_output_dim(
        self, dimension, padding, dilation, kernel_size, stride
    ):
        r"""Calculates the output height and width based on the input
        height and width to the convolution layer.

        ref: https://pytorch.org/docs/master/nn.html#torch.nn.Conv2d
        """
        assert len(dimension) == 2
        out_dimension = []
        for i in range(len(dimension)):
            out_dimension.append(
                int(
                    np.floor(
                        (
                            (
                                dimension[i]
                                + 2 * padding[i]
                                - dilation[i] * (kernel_size[i] - 1)
                                - 1
                            )
                            / stride[i]
                        )
                        + 1
                    )
                )
            )
        return tuple(out_dimension)

    def layer_init(self):
        for layer in self.cnn:
            if isinstance(layer, (nn.Conv2d, nn.Linear)):
                nn.init.kaiming_normal_(
                    layer.weight, nn.init.calculate_gain("relu")
                )
                if layer.bias is not None:
                    nn.init.constant_(layer.bias, val=0)

    @property
    def is_blind(self):
        return self._n_input_rgb + self._n_input_depth == 0

    @property
    def input_shape(self):
        return self._input_shape

    @property
    def output_shape(self):
        return 1, self._output_size

    @property
    def feature_dims(self):
        return self._output_size

    def forward(self, observations):
        cnn_input = []
        if self._n_input_rgb > 0:
            rgb_observations = observations["rgb"]
            # permute tensor to dimension [step*batch x CHANNEL x HEIGHT X WIDTH]
            rgb_observations = rgb_observations.permute(0, 3, 1, 2).contiguous()
            # rgb_observations = rgb_observations / 255.0  # normalize RGB
            cnn_input.append(rgb_observations)

        if self._n_input_depth > 0:
            depth_observations = observations["depth"]
            # permute tensor to dimension [step*batch x CHANNEL x HEIGHT X WIDTH]
            depth_observations = depth_observations.permute(0, 3, 1, 2).contiguous()
            cnn_input.append(depth_observations)
            
        if self._n_input_map > 0:
            map_observations = observations["floorplan_map"]
            # permute tensor to dimension [step*batch x CHANNEL x HEIGHT X WIDTH]
            map_observations = map_observations.permute(0, 3, 1, 2).contiguous()
            cnn_input.append(map_observations)
        cnn_input = torch.cat(cnn_input, dim=1)
        feat = self.cnn(cnn_input)
        feat = F.adaptive_avg_pool2d(feat, 1)
        for mod in self.scaler:
            feat = mod(feat)
        return feat


def convert_semantics_to_rgb(semantics):
    r"""Converts semantic IDs to RGB images.
    """
    semantics = semantics.long() % 40
    mapping_rgb = torch.from_numpy(d3_40_colors_rgb).to(semantics.device)
    semantics_r = torch.take(mapping_rgb[:, 0], semantics)
    semantics_g = torch.take(mapping_rgb[:, 1], semantics)
    semantics_b = torch.take(mapping_rgb[:, 2], semantics)
    semantics_rgb = torch.stack([semantics_r, semantics_g, semantics_b], -1)

    return semantics_rgb