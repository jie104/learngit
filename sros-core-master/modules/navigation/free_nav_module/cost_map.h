//
// Created by yj on 20-8-26.
//

#ifndef SROS_COST_MAP_H
#define SROS_COST_MAP_H


class CostMap {
public:
    CostMap(unsigned int cells_size_x, unsigned int cells_size_y,double resolution,
                     double origin_x,double origin_y, unsigned char default_value);


    void initMaps(unsigned int size_x,unsigned int size_y);

    void resetMaps();

    void deleteMaps();

    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;

    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;

    void setCost(unsigned int mx, unsigned int my, unsigned char cost);

    unsigned char getCost(unsigned int mx, unsigned int my) const;

    double getOriginX() const;

    double getOriginY() const;

    double getResolution() const;


    inline unsigned int getIndex(unsigned int mx, unsigned int my) const
    {
        return my * size_x_ + mx;
    }

    inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const
    {
        my = index / size_x_;
        mx = index - (my * size_x_);
    }


private:
    unsigned int size_x_;
    unsigned int size_y_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    unsigned char* costmap_;
    unsigned char default_value_;

};


#endif //SROS_COST_MAP_H
