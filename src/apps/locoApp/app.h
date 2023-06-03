#pragma once

#include <crl-basic/gui/application.h>

#include "loco/controller/KinematicTrackingController.h"
#include "loco/controller/RunningController.h"
#include "loco/planner/GaitPlanner.h"
#include "loco/planner/SimpleLocomotionTrajectoryPlanner.h"
#include "menu.h"

namespace locoApp {

class App : public crl::gui::ShadowApplication {
public:
    std::shared_ptr<crl::loco::RBJoint>  plottingJoint;

    App() : crl::gui::ShadowApplication("Locomotion App") {
        this->showConsole = true;
        this->automanageConsole = true;
        this->showPlots = true;
        this->show_world_frame = false;

        // setup
        setupRobotAndController();
    }

    ~App() override = default;

    void process() override {
        // add gait plan
        controller_->planner->appendPeriodicGaitIfNeeded(gaitPlanner_->getPeriodicGait(robot_));

        double simTime = 0;
        while (simTime < 1.0 / targetFramerate) {
            simTime += dt;
            controller_->computeAndApplyControlSignals(dt);
            controller_->advanceInTime(dt);
        }

        // generate motion plan
        controller_->generateMotionTrajectories();

        // adjust light and camera
        const auto &center = robot_->getTrunk()->getWorldCoordinates(crl::P3D());
        float yTarget = decltype(ground)::getHeight(center) + 0.9;
        if (followRobotWithCamera) {
            camera.target.x = (float)center.x;
            camera.target.y = yTarget;
            camera.target.z = (float)center.z;
        }
        light.target.x() = center.x;
        light.target.y() = yTarget;
        light.target.z() = center.z;
    }

    void restart() override {
        setupRobotAndController();
    }

    void drawObjectsWithShadows(const crl::gui::Shader &shader) override {
        ShadowApplication::drawObjectsWithShadows(shader);
    }

    void drawShadowCastingObjects(const crl::gui::Shader &shader) override {
        robot_->draw(shader);
    }

    void drawObjectsWithoutShadows(const crl::gui::Shader &shader) override {
        robot_->draw(shader);

        if (drawDebugInfo)
            controller_->drawDebugInfo(&basicShader);
    }

    bool keyPressed(int key, int mods) override {
        if (key == GLFW_KEY_SPACE) {
            processIsRunning = !processIsRunning;
        }
        if (key == GLFW_KEY_ENTER) {
            if (!processIsRunning)
                process();
        }

        // joystick command
        bool dirty = false;
        if (key == GLFW_KEY_UP) {
            planner_->speedForward += 0.1;
            dirty = true;
        }
        if (key == GLFW_KEY_DOWN) {
            planner_->speedForward -= 0.1;
            dirty = true;
        }
        if (key == GLFW_KEY_LEFT) {
            planner_->turningSpeed += 0.1;
            dirty = true;
        }
        if (key == GLFW_KEY_RIGHT) {
            planner_->turningSpeed -= 0.1;
            dirty = true;
        }

        if (dirty) {
            planner_->appendPeriodicGaitIfNeeded(gaitPlanner_->getPeriodicGait(robot_));
            controller_->generateMotionTrajectories();
            return true;
        }

        return false;
    }

    template <typename T>
    void drawComboMenu(const std::string &menuName, const std::vector<T> &options, uint &selected) {
        if (ImGui::BeginCombo(menuName.c_str(), options[selected].name.c_str())) {
            for (uint n = 0; n < options.size(); n++) {
                bool is_selected = (selected == n);
                if (ImGui::Selectable(options[n].name.c_str(), is_selected)) {
                    selected = n;
                    setupRobotAndController();
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
    }

    void drawImGui() override {
        crl::gui::ShadowApplication::drawImGui();

        ImGui::Begin("Main Menu");
        ImGui::Checkbox("Follow Robot with Camera", &followRobotWithCamera);
        if (ImGui::CollapsingHeader("Character")) {
            drawComboMenu("Model##character", modelOptions, selectedModel);
            ImGui::Checkbox("Use Flat Terrain", &crl::gui::SimpleGroundModel::isFlat);
        }

        if (ImGui::CollapsingHeader("Draw")) {
            if (ImGui::Checkbox("Show meshes", &robot_->showMeshes)) {
                robot_->showSkeleton = !robot_->showMeshes;
            }
            ImGui::Checkbox("Show end effectors", &robot_->showEndEffectors);
            ImGui::Checkbox("Draw debug info", &drawDebugInfo);
        }

        if (ImGui::CollapsingHeader("Gait Parameters")) {
            ImGui::SliderDouble("Cycle Length", &modelOptions[selectedModel].cycleLength, 0.5, 1.5, "%f", ImGuiSliderFlags_AlwaysClamp);
            ImGui::SliderDouble("Swing Start", &modelOptions[selectedModel].swingStart, modelOptions[selectedModel].stanceStart, modelOptions[selectedModel].heelStrikeStart, "%f", ImGuiSliderFlags_AlwaysClamp);
            ImGui::SliderDouble("Heel Strike Start", &modelOptions[selectedModel].heelStrikeStart, modelOptions[selectedModel].swingStart, 1.0, "%f", ImGuiSliderFlags_AlwaysClamp);
        }

        ImGui::End();

        planner_->visualizeContactSchedule();
        planner_->visualizeParameters();
    }

    void drawImPlot() override {
        crl::gui::ShadowApplication::drawImPlot();

        ImGui::SliderDouble("Delta time", &dt, 1.0/1080, 1.0/30, "%f", ImGuiSliderFlags_AlwaysClamp);
        ImGui::SliderInt("Target Framerate", &targetFramerate, 1, 240, "%d", ImGuiSliderFlags_AlwaysClamp);

        const double gaitCycleLength = controller_->cycleLength;

        static double tPrev = controller_->t;


        const auto numSamples = static_cast<size_t>(60*gaitCycleLength);

        static size_t idx = 0;
        if(controller_->t != tPrev)
            idx = (idx + 1) % numSamples;
        tPrev = controller_->t;


        static std::vector<double> x_vals(numSamples), y_vals(numSamples),
                                   velocities(numSamples), accelerations(numSamples);



        const auto joints = [&](){

            using jointPtr = std::shared_ptr<crl::loco::RBJoint>;
            using rbPtr = std::shared_ptr<crl::loco::RB>;
            std::vector<jointPtr> jointVec{};

            std::function<void(const rbPtr&)> addChildJoints = [&addChildJoints, &jointVec](const rbPtr& rb){
                if(!rb) return;

                for(const auto& joint : rb->cJoints){
                    //continue if RB is already in list
                    if(!(std::find(jointVec.begin(), jointVec.end(), joint) == jointVec.end())) continue;
                    jointVec.push_back(joint);
                    addChildJoints(joint->child);
                }
            };
            addChildJoints(robot_->getTrunk());

            return jointVec;
        }();


        ImGui::Begin("Plots");
        if (ImGui::BeginMenu("Select Joint"))
        {
            for(const auto &j : joints){
                if(ImGui::MenuItem(j->name.c_str())){ plottingJoint = j;}
            }
            ImGui::EndMenu();
        }


        x_vals[idx] = 100 * fmod(controller_->t, gaitCycleLength)/gaitCycleLength;
        y_vals[idx] = plottingJoint->getCurrentJointAngle() * 180.0 * 1/3.14159265358979323846;

        velocities[idx] = (y_vals[(idx + 1)%numSamples] - y_vals[(idx + numSamples - 1)%numSamples])/(2 * dt);
//        accelerations[idx] = (velocities[(idx + 1)%numSamples] - velocities[(idx + numSamples - 1)%numSamples])/(2 * dt);


        if (ImPlot::BeginPlot("Joint Angle Plot")) {

            double plotMax = *std::max_element(std::begin(y_vals), std::end(y_vals));
            double plotMin = *std::min_element(std::begin(y_vals), std::end(y_vals));

//            double plotMax = std::max(
//                    *std::max_element(std::begin(y_vals), std::end(y_vals)), //std::max(
//                    *std::max_element(std::begin(velocities), std::end(velocities))
////                    *std::max_element(std::begin(accelerations), std::end(accelerations)))
//                ), plotMin = std::min(
//                    *std::min_element(std::begin(y_vals), std::end(y_vals)), //std::min(
//                    *std::min_element(std::begin(velocities), std::end(velocities))
////                    *std::min_element(std::begin(accelerations), std::end(accelerations)))
//                );

//            std::cout << *std::min_element(std::begin(y_vals), std::end(y_vals)) << '/'
//                      << *std::min_element(std::begin(velocities), std::end(velocities)) << '/'
//                      << *std::min_element(std::begin(accelerations), std::end(accelerations)) << '\n';

            ImPlot::SetupAxesLimits(0, 100, plotMin - 10, plotMax + 10, ImPlotCond_Always);

            ImPlot::PlotScatter(plottingJoint->name.c_str(), x_vals.data(), y_vals.data(), static_cast<int>(numSamples));
//            ImPlot::PlotScatter("Velocity", x_vals.data(), velocities.data(), numSamples);
//            ImPlot::PlotScatter("Acceleration", x_vals, accelerations, numSamples);


            double current_x[] = {x_vals[idx]};

            ImPlot::PlotVLines("Current time", current_x, 1);


            ImPlot::EndPlot();
        }

        // here, you can draw plots
        ImGui::End();
    }

    bool drop(int count, const char **fileNames) override {
        return true;
    }

private:
    void setupRobotAndController() {
        auto &m = modelOptions[selectedModel];
        const char *rbsFile = m.filePath.c_str();
        robot_ = std::make_shared<crl::loco::LeggedRobot>(rbsFile);
        //TODO make + crl::gui::SimpleGroundModel::getHeight(crl::P3D(0, 0, 0)) work here
        robot_->setRootState(crl::P3D(0, m.baseTargetHeight, 0));
        
        gaitPlanner_ = std::make_shared<crl::loco::BipedalGaitPlanner>();

        // add legs
        for (const auto& leg : m.legs) {
            robot_->addLimb(leg.first, leg.second);
        }

        unsigned n_limbs = robot_->getLimbCount();
        std::cout << "Printing " << n_limbs << " Limbs:\n";
        for (unsigned i = 0; i < n_limbs; ++i) {
            auto limb = robot_->getLimb(i);
            std::cout << "Limb " << i << " with name " << limb->name << ":\n";
            for (auto &joint : limb->jointList) {
                std::cout << "Joint: " << joint->name << "\n";
            }
            std::cout << "eeRB:" << limb->eeRB->name << "[" << limb->getEEWorldPos().x << ", " << limb->getEEWorldPos().y << ", " << limb->getEEWorldPos().z
                      << "]"
                      << "\n";
        }
        std::cout << std::endl;

        // setup planner and controller
        planner_ = std::make_shared<crl::loco::SimpleLocomotionTrajectoryPlanner>(robot_);
        planner_->trunkHeight = m.baseTargetHeight;
        if (m.type == ModelOption::Type::BOB) {
            controller_ = std::make_shared<crl::loco::KinematicTrackingController>(
                planner_,
                m.cycleLength,
                m.stanceStart,
                m.swingStart,
                m.heelStrikeStart
            );
        } else if (m.type == ModelOption::Type::RUN) {
            controller_ = std::make_shared<crl::loco::RunningController>(
                planner_,
                m.cycleLength,
                m.stanceStart,
                m.swingStart,
                m.heelStrikeStart
            );
        }

        // generate plan
        planner_->appendPeriodicGaitIfNeeded(gaitPlanner_->getPeriodicGait(robot_));
        controller_->generateMotionTrajectories();

        plottingJoint = robot_->getJointByName("rAnkle_1");
    }

public:
    // simulation

    std::shared_ptr<crl::loco::LeggedRobot> robot_ = nullptr;
    std::shared_ptr<crl::loco::GaitPlanner> gaitPlanner_ = nullptr;
    std::shared_ptr<crl::loco::LocomotionTrajectoryPlanner> planner_ = nullptr;
    std::shared_ptr<crl::loco::LocomotionController> controller_ = nullptr;

    // parameters
    double dt = 1 / 60.0;

    // options
    uint selectedModel = 0;
    bool followRobotWithCamera = true;
    bool drawDebugInfo = true;
};

}  // namespace locoApp
