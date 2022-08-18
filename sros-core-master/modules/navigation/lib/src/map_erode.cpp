#include "../include/map_erode.h"
#include "ipl/cimgboard.hh"

ipl::Region loadImage(sros::map::PyramidNavMap_ptr map_data, int x, int y) {
    unsigned char *tt;
    int map_data_length = x * y;
    tt = new unsigned char[map_data_length];
//    for (int i = 0; i < map_data_length; ++i) {
//        if (map_data->isOccInGridIndex(i))
//            tt[i] = sros::map::OCC;
//        else
//            tt[i] = sros::map::FREE;
//    }
    map_data->cpToArray(tt, sros::map::FREE, sros::map::OCC);
    ipl::PictImage img(tt, x, y);
    ipl::Region reg = img.binarize(sros::map::OCC, sros::map::T_VALUE);
    delete[] tt;

    return reg;
}

void writeBackToMap(ipl::PictImage bin, sros::map::PyramidNavMap_ptr map_data) {
    int i = 0;
    auto start = bin.begin(), end = bin.end();
    map_data->clear();
    for (; start != end; ++start) {
        if (*start == sros::map::OCC)
            map_data->setGridIndexValue(i,1);
        i++;
        //        else
//            map_data->setGridIndexValue(i++,0);
    }
}

void inflationStaticMap(sros::map::PyramidNavMap_ptr navigation_map,
                        sros::map::PyramidNavMap_ptr inflation_map_data,
                        int width, int height, int inflation_factor,
                        StructuringElementType kenel_type, int origin_map) {
    ipl::Region reg, B8, morphOutput8;
    if (origin_map == 1) {
        reg = loadImage(navigation_map, width, height);
    } else if (origin_map == 2) {
        reg = loadImage(inflation_map_data, width, height);
    }
    B8 = ipl::Region::generateStructuringElement((ipl::Region::StructuringElement) kenel_type, inflation_factor);
    morphOutput8 = reg.dilate(B8);
    ipl::PictImage bin(width, height);
    bin.fill(sros::map::FREE);
    bin.setRoi(morphOutput8);
    bin.fill(sros::map::OCC);
    writeBackToMap(bin, inflation_map_data);
}

Obstacle_vector inflationObstacles(Obstacle_vector obstacles, int width, int height,
                                              StructuringElementType kenel_type, int inflation_factor) {
    int map_data_length, index, i, j;
    map_data_length = width * height;
    unsigned char *tt = new unsigned char[map_data_length];
    ipl::Region reg, B8, morphOutput8;
    Obstacle_vector::iterator it;
    sros::map::Point p;

    memset(tt, sros::map::FREE, map_data_length * sizeof(unsigned char));
    for (it = obstacles.begin(); it != obstacles.end(); ++it) {
        p = *it;
        index = p.y * width + p.x;
        if (index < 0 || index > map_data_length)
            continue;
        tt[index] = sros::map::OCC;
    }
    ipl::PictImage img(tt, width, height);
    reg = img.binarize(sros::map::OCC, sros::map::T_VALUE);
    delete[] tt;

    B8 = ipl::Region::generateStructuringElement((ipl::Region::StructuringElement) kenel_type, inflation_factor);
    morphOutput8 = reg.dilate(B8);
    ipl::PictImage bin(width, height);
    bin.fill(sros::map::FREE);
    bin.setRoi(morphOutput8);
    bin.fill(sros::map::OCC);

    Obstacle_vector obs;
    for (i = 0; i < width; ++i) {
        for (j = 0; j < height; ++j) {
            if (bin(i, j) == sros::map::OCC) {
                sros::map::Point te(i , j);
                obs.push_back(te);
            }
        }
    }
    return obs;
}