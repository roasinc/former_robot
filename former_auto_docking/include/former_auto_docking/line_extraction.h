#ifndef LINE_EXTRACTION_H
#define LINE_EXTRACTION_H

#include <cmath>
#include <vector>
#include <boost/array.hpp>
#include <Eigen/Dense>
#include "former_auto_docking/utilities.h"
#include "former_auto_docking/line.h"

class LineExtraction
{

    public:
        LineExtraction();
        ~LineExtraction();

    public:
        void extractLines(std::vector<Line>&, double max_length=0.0);
        void setCachedData(const std::vector<double>&, const std::vector<double>&, const std::vector<double>&, const std::vector<unsigned int>&);
        void setRangeData(const std::vector<double>&);

        void setBearingVariance(double);
        void setRangeVariance(double);
        void setLeastSqAngleThresh(double);
        void setLeastSqRadiusThresh(double);
        void setMaxLineGap(double);
        void setMinLineLength(double);
        void setMinLinePoints(unsigned int);
        void setMinRange(double);
        void setMaxRange(double);
        void setMinSplitDist(double);
        void setOutlierDist(double);

    private:
        double chiSquared(const Eigen::Vector2d&, const Eigen::Matrix2d&, const Eigen::Matrix2d&);
        double distBetweenPoints(unsigned int index_1, unsigned int index_2);
        void filterCloseAndFarPoints();
        void filterOutlierPoints();
        void filterLines();
        void mergeLines();
        void split(const std::vector<unsigned int>&);

    private:
        CachedData c_data_;
        RangeData r_data_;
        Params params_;
        std::vector<unsigned int> filtered_indices_;
        std::vector<Line> lines_;
};

#endif //LINE_EXTRACTION_H