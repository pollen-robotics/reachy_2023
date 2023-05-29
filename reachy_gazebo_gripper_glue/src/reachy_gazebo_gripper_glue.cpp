#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

/*
 * I'm glue da ba dee da ba daa
 * Da ba dee da ba daa, da ba dee da ba daa, da ba dee da ba daa
 * I'm glue da ba dee da ba daa
 * Da ba dee da ba daa, da ba dee da ba daa, da ba dee da ba daa
 */

namespace gazebo
{
    class ReachyGazeboGripperGlue : public WorldPlugin
    {
    public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
            this->world = _world;
            this->initialized = false;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ReachyGazeboGripperGlue::OnUpdate, this));
            this->object_links = std::vector<physics::LinkPtr>();
            gzdbg << "ReachyGazeboGripperGlue loaded" << std::endl;
        }

        void UpdateModelsAndLinks()
        {
            auto models = this->world->Models();

            for (const auto& model : models)
            {
                if (model->GetName() != "reachy")
                {
                    auto links = model->GetLinks();

                    if (!links.empty())
                    {
                        auto link = links[0];

                        if (std::find(this->object_links.begin(), this->object_links.end(), link) == this->object_links.end())
                        {
                            this->object_links.push_back(link);
                        }
                    }
                }
            }
        }

        void OnUpdate()
        {
            if (!this->initialized)
            {
                this->initialized = InitModelsAndLinks();
                if (!this->initialized)
                {
                    return;
                }
            }

            this->UpdateModelsAndLinks();
            double threshold_distance = 0.1;
            double gripper_opening = this->GetDistance(this->robot_link_finger, this->robot_link_thumb);

            for (const auto& object_link : this->object_links)
            {
                double distance_thumb = this->GetDistance(this->robot_link_thumb, object_link);
                double distance_finger = this->GetDistance(this->robot_link_finger, object_link);
                bool thumb_contact = CheckContact(this->robot_link_thumb, object_link);
                bool finger_contact = CheckContact(this->robot_link_finger, object_link);

                // LOOK AT ME, I'M THE GLUE NOW
                if (
                ((distance_thumb < threshold_distance && distance_finger < threshold_distance && gripper_opening < 0.1)
                || ((thumb_contact || finger_contact) && gripper_opening < 0.1))
                && !this->joint)
                {
                    this->joint = this->world->Physics()->CreateJoint("revolute", robot_link_thumb->GetModel());
                    this->joint->SetName("glue");
                    this->joint->Load(this->robot_link_thumb, object_link, ignition::math::Pose3d());
                    this->joint->Init();

                    std::cout << "Joint created with " << object_link->GetModel()->GetName() << std::endl;
                }
            }
            // TODO remove joint in list when they are removed in gazebo

            if (gripper_opening >= 0.1 && this->joint)
            {
                this->joint->Detach();
                this->joint.reset();

                std::cout << "Joint removed" << std::endl;
            }
        }

        double GetDistance(physics::LinkPtr link1, physics::LinkPtr link2)
        {
            auto pos1 = link1->WorldCoGPose().Pos();
            auto pos2 = link2->WorldCoGPose().Pos();
            return pos1.Distance(pos2);
        }

        bool CheckContact(physics::LinkPtr link1, physics::LinkPtr link2)
{
    auto contacts = this->world->Physics()->GetContactManager()->GetContacts();

    for (const auto &contact : contacts)
    {
        if ((contact->collision1->GetLink() == link1 && contact->collision2->GetLink() == link2) ||
            (contact->collision1->GetLink() == link2 && contact->collision2->GetLink() == link1))
        {
            return true;
        }
    }

    return false;
}


    private:
        bool InitModelsAndLinks()
        {
            physics::ModelPtr robot_model = this->world->ModelByName("reachy");

            if (!robot_model)
            {
                gzerr << "Model(s) not found" << std::endl;
                return false;
            }

            this->robot_link_thumb = robot_model->GetLink("reachy::r_gripper_thumb");
            this->robot_link_finger = robot_model->GetLink("reachy::r_gripper_finger");

            if (!this->robot_link_thumb || !this->robot_link_finger)
            {
                gzerr << "Link(s) not found" << std::endl;
                return false;
            }

            return true;
        }

        physics::WorldPtr world;
        physics::LinkPtr robot_link_thumb;
        physics::LinkPtr robot_link_finger;
        physics::JointPtr joint;
        event::ConnectionPtr updateConnection;
        bool initialized;
        std::vector<physics::LinkPtr> object_links;
    };

    GZ_REGISTER_WORLD_PLUGIN(ReachyGazeboGripperGlue)
}