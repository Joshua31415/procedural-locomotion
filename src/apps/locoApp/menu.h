#include <string.h>

#include <string>
#include <vector>

#include "crl-basic/utils/mathDefs.h"

namespace locoApp {

struct ModelOption {
    enum class Type {
        BOB = 0,
        RUN = 1,
    };

    Type type;
    std::string name;
    std::string filePath;
    std::vector<std::pair<std::string, std::string>> legs;
    double baseTargetHeight;
    double cycleLength;
    double stanceStart;
    double swingStart;
    double heelStrikeStart;
};

const std::vector<ModelOption> modelOptions = {
    {
        ModelOption::Type::BOB,                 //
        "Bob",                                  //
        CRL_DATA_FOLDER "/robots/bob/bob.rbs",  //
        {
            {"rFoot", "rFoot"}, 
            {"lFoot", "lFoot"},
            {"rLowerLeg", "rLowerLeg"},
            {"lLowerLeg", "lLowerLeg"},
        },
        0.9,   // base height
        0.9,
        0.0,
        0.5,
        0.9,
    },
    {
        ModelOption::Type::RUN,                         //
        "Running Bob",                                          //
        CRL_DATA_FOLDER "/robots/bob/bob_running.rbs",  //
        {
            {"rFoot", "rToes"},
            {"lFoot", "lToes"},
            {"rLowerLeg", "rFoot"},
            {"lLowerLeg", "lFoot"},
        },
        0.88,   // base height
        60.0 / 75.0,
        0.0,
        0.35,
        0.95,
    },
};

}  // namespace locoApp
