
/* BResultAnalysis 20/04/2026

 $$$$$$$$$$$$$$$$$$$$$$$$$
 $   BResultAnalysis.h   $
 $$$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Quick and dirty pretty print results
 
*/



#ifndef __BRESULTANALYSIS_H__
#define __BRESULTANALYSIS_H__

#include <cmath>
#include <iomanip>
#include <algorithm>


class BResults 
{
public:
    using BDataMap = std::map<std::string, std::vector<double>>;

    BResults(void) = default;
    explicit BResults(BDataMap data, const std::string &vt) : m_data(data), m_valuetype(vt) {}
    ~BResults(void) = default;
    
    
    const BDataMap& 
    data(void) const { return m_data; }

    void 
    print(void) const 
    {
        std::cout << "----------------------------------" << std::endl;
        std::cout << std::left << std::setw(20) << "Body" << m_valuetype << std::endl;
        std::cout << "----------------------------------" << std::endl;
        for (const auto& [name, values] : m_data) 
        {
            std::cout << std::left << std::setw(20) << name << " ";
            for (double val : values) 
            {
                std::cout << std::fixed << std::setw(14) << val;
            }
            std::cout << "\n";
        }
    }


    double 
    max_error(const BResults& other) const 
    {
        const BDataMap& otherData = other.data();
        if (m_data.size() != otherData.size()) 
            return -1.0;

        double maxErr = 0.0;
        for (const auto& [name, val1] : m_data) 
        {
            auto it = otherData.find(name);
            
            if (it == otherData.end() || val1.size() != it->second.size()) 
            {
                return -1.0;
            }

            const auto& val2 = it->second;
            for (size_t i = 0; i < val1.size(); ++i) 
            {
                maxErr = std::max(maxErr, std::abs(val1[i] - val2[i]));
            }
        }
        return maxErr;
    }

private:
    std::string m_valuetype;
    BDataMap m_data;
};


class BResultAnalysis 
{
public:
 
    static BResults 
    extract_acc(const BModel& model, const BModelState& state) 
    {
        BResults::BDataMap results;
        int start = 0;

        for (int i = 1; i < model.numBody(); ++i) 
        {
            int dof = model.joint(i).DoFCount();
            std::string name = model.getBodyName(model.body(i).getId());

            std::vector<double> joint_accels;
            joint_accels.reserve(dof);

            for (int j = 0; j < dof; ++j) 
            {
                joint_accels.push_back((double) state.qddot[start + j]);
            }

            results[name] = joint_accels;
            start += dof;
        }

        return BResults(results, "Acceleration");
    }
    static BResults 
    extract_tau(const BModel& model, const BModelState& state) 
    {
        BResults::BDataMap results;
        int start = 0;

        for (int i = 1; i < model.numBody(); ++i) 
        {
            int dof = model.joint(i).DoFCount();
            std::string name = model.getBodyName(model.body(i).getId());

            std::vector<double> joint_tau;
            joint_tau.reserve(dof);

            for (int j = 0; j < dof; ++j) 
            {
                joint_tau.push_back((double) state.tau[start + j]);
            }

            results[name] = joint_tau;
            start += dof;
        }

        return BResults(results, "Tau");
    }
};


#endif


