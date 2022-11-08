#ifndef LINE_EXTRACTION_LINE_H
#define LINE_EXTRACTION_LINE_H

#include <vector>
#include <boost/array.hpp>
#include "former_auto_docking/utilities.h"

class Line
{
    public:
        Line(const CachedData&, const RangeData&, const Params&, std::vector<unsigned int>);
        Line(double angle, double radius, const boost::array<double, 4> &covariance,
            const boost::array<double, 2> &start, const boost::array<double, 2> &end,
            const std::vector<unsigned int> &indices);
        ~Line();

    public:
        double getAngle() const;
        const boost::array<double, 4>& getCovariance() const;
        const boost::array<double, 2>& getEnd() const;
        const std::vector<unsigned int>& getIndices() const;
        double getRadius() const;
        const boost::array<double, 2>& getStart() const;

        double distToPoint(unsigned int);
        void endpointFit();
        void leastSqFit();
        double length() const;
        unsigned int numPoints() const;
        void projectEndpoints();

    private:
        void angleFromEndpoints();
        void angleFromLeastSq();
        double angleIncrement();
        void calcCovariance();
        void calcPointCovariances();
        void calcPointParameters();
        void calcPointScalarCovariances();
        void radiusFromEndpoints();
        void radiusFromLeastSq();

    private:
        CachedData c_data_;
        RangeData r_data_;
        Params params_;
        double p_rr_;
        double angle_;
        double radius_;
        boost::array<double, 4> covariance_;
        boost::array<double, 2> start_;
        boost::array<double, 2> end_;
        std::vector<unsigned int> indices_;
        PointParams p_params_;
        std::vector<double> point_scalar_vars_;
        std::vector<boost::array<double, 4> > point_covs_;



};

#endif