#include <string.h>

#include <string>
#include <vector>

#include "crl-basic/utils/mathDefs.h"

namespace locoApp {

struct ModelOption {
    enum class Type {
        BOB = 0,
        DOG = 1,
    };

    Type type;
    std::string name;
    std::string filePath;
    std::vector<std::pair<std::string, std::string>> legs;
    double baseTargetHeight;
    double swingFootHeight;
};

const std::vector<ModelOption> modelOptions = {
    {
        ModelOption::Type::BOB,                 //
        "Bob",                                  //
        CRL_DATA_FOLDER "/robots/bob/bob_running.rbs",  //
        {
            {"rFoot", "rToes"}, // {"rFoot", "rToes"} 
            {"lFoot", "lToes"},
            {"rLowerLeg", "rFoot"},
            {"lLowerLeg", "lFoot"},
//            {"lHand", "lHand"},
//            {"rHand", "rHand"},
           // {"lLowerArm", "lLowerArm"},
            //{"rLowerArm", "rLowerArm"},
            //{"lUpperArm", "lUpperArm"},
            //{"rUpperArm", "rUpperArm"},

        },
        0.9,   //
        0.15,  //
    },
    {
        ModelOption::Type::DOG,                 //
        "Dog",                                  //
        CRL_DATA_FOLDER "/robots/dog/dog.rbs",  //
        {
            {"fl", "tibia_0"},  //
            {"hl", "tibia_1"},  //
            {"fr", "tibia_2"},  //
            {"hr", "tibia_3"},  //
        },
        0.437,  //
        0.1,    //
    },
};

}  // namespace locoApp
