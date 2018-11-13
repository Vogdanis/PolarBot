//
//  Image.cpp
//  PolarBot
//
//  Created by Eyecon on 05/11/2018.
//
//

#include "Image.hpp"

void ImageClass::setup(){
    image.loadImage("test.jpg");
}

void ImageClass::draw(){
    image.draw(100,100);
}
