#include <ros/ros.h>

#include "pr2_cooking_demo/cooking.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pr2_cooking_demo");
    ros::NodeHandle n("~");
    int state;

    Cooking cooking(&n);
    // cooking.say("Let's make a fruit salad");

    ros::Rate r(30);

    while (ros::ok()) {
        ROS_INFO_THROTTLE(60, "Waiting!");
        ros::spinOnce();
        state = cooking.get_state();
        switch (state) {  
            case 0: // Look around and talk
                ros::Duration(5).sleep();
                cooking.look_at("table");
                ros::Duration(3).sleep();
                cooking.look_at("banana");
                cooking.look_at("apple1");
                cooking.look_at("bowl1");

                cooking.say(cooking.banana_and_apple_phrase);
                
                ros::Duration(8).sleep();
                
                cooking.say(cooking.use_bowl_phrase);

                ros::Duration(8).sleep();

                cooking.set_state(1);  
            case 1: // mix bananna with apple in bowl 1
                cooking.say(cooking.ok_making_salad_phrase);
                cooking.combine_fruits("banana","apple1","bowl1");

                ros::Duration(8).sleep();

                cooking.set_state(2);
                break;
            case 2: // make second salad; ask about missing banana
                cooking.look_at("orange");
                cooking.look_at("apple2");
                cooking.look_at("bowl2");
                cooking.say(cooking.no_banana_use_orange_phrase);

                ros::Duration(8).sleep();

                cooking.set_state(3);
                break;
            case 3: // mix apple with orange in bowl 2
                cooking.say(cooking.ok_making_salad_phrase);
                cooking.combine_fruits("orange","apple2","bowl2");
                cooking.set_state(-1);
                break;
            default: // Listen (do nothing)
                cooking.look_at("person");
                ros::spinOnce();
        }
        r.sleep();
    }

    ros::shutdown();
    return 0;
}