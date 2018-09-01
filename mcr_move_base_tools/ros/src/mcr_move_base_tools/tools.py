import os
from dynamic_reconfigure.client import Client
import rospy
import rosparam
import re
import rospkg

class PlannerUpdater:
    def __init__(navigation_server = "/move_base/", config_package = "mdr_2dnav"):

        rospy.init_node("dynamic_reconfigure_planners")
        self.navigation_server = navigation_server
        #Dynamic Client Instatinng
        self.dyn_client = Client(navigation_server, None)

        if not rospy.has_param(navigation_server + "base_global_planner"):
            return False

        if not rospy.has_param(navigation_server + "base_local_planner"):
            return False

        #Getting Data of current planners
        #For Global Planner
        current_global_planner = rosparam.get_param(navigation_server + "base_global_planner")
        current_global_planner_ns = current_global_planner.split('/')[0]
        self.current_global_planner_name = current_global_planner.split('/')[1]

        #For Local Planner
        current_local_planner = rosparam.get_param(navigation_server + "base_local_planner")
        current_local_planner_ns = current_local_planner.split('/')[0]
        self.current_local_planner_name = current_local_planner.split('/')[1]

        #TODO Maybe not needed if config files are added
        rospack = rospkg.RosPack()
        self.config_path = rospack.get_path(config_package)

        #Get planners information running rospack plugin on terminal/ Retunr Name/plugin path
        plugins = os.popen("rospack plugins --attrib=plugin nav_core").read()
        plugins = plugins.splitlines()

        plugin_name_type = list()

        #Parsing information from Plugin Paths
        #Name / Type / plugin_base_type
        for i in plugins:
            j = i.split()
            c_file = open(j[1],"r")
            #Flags for plugin name, type and base_class_type
            flags = [False,False,False]
            #iterating over lines
            for line in c_file:
                #Find indexes
                index_name = line.find("name")
                index_type = line.find("type")
                base_class_index = line.find("base_class_type")

                if index_name > 0 and not flags[0]:
                    name = re.findall(r'"(.*?)"', line[index_name:])[0]
                    flags[0] = True
                if index_type>0 and not flags[1]:
                    plugin_type = re.findall(r'"(.*?)"', line[index_type:])[0]
                    flags[1] = True
                if base_class_index > 0 and not flags[2]:
                    plugin_base_type = re.findall(r'"(.*?)"', line[base_class_index:])[0]
                    flags[2] = True

                if all(flags):
                    plugin_name_type.append([name, plugin_type, plugin_base_type])
                    flags = [False,False,False]

        #Parsing
        self.available_global_planners = list()
        self.available_local_planners = list()


        #Where just interested on Global and Local planners
        #Recovery Behaviors are ignored
        for n_t in plugin_name_type:
            print n_t[0], n_t[2]
            if n_t[2] == "nav_core::BaseGlobalPlanner":
                self.available_global_planners.append(n_t[0])

            if n_t[2] == "nav_core::BaseLocalPlanner":
                self.available_local_planners.append(n_t[0])


        def getAvailableGlobalPlanners(self):
            return self.available_global_planners

        def getAvailableLocalPlanners(self):
            return self.available_local_planners

        def deleteOldParams(self):
            rospy.logwarn("Deleting Old Params")
            current_planners = [self.current_global_planner_name,
                                self.current_local_planner_name]

            for planner in current_planners:
                ns = self.navigation_server + planner
                old_parameters = rosparam.get_param(ns)
                for param in old_parameters:
                    rosparam.delete_param(ns+'/'+param)

        def addNewParams(self, new_namespace):
            rospy.loginfo("Adding New Params")
            param_file = self.config_path+'/'+ new_namespace + '.yaml'
            new_config_file = rosparam.load_file(param_file)

            for params,ns in new_config_file:
                print ns, params
                rosparam.upload_params(navigation_server + ns,params)

        def updatePlanner(self, new_global_planner_ns=None, new_local_planner_ns=None, new_config):
            if new_global_planner_ns is not None:
                planner_updater.deleteOldParams(self.current_global_planner_name)
                planner_updater.addNewParams(new_global_planner_ns)
            if new_local_planner_ns is not None:
                planner_updater.deleteOldParams(self.current_local_planner_name)
                planner_updater.addNewParams(new_locbal_planner_ns)

            self.dyn_client.update_configuration(new_config)
