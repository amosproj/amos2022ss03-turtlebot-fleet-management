import sys
import time

import mqtt
import vda5050
from models.Order import OrderType

""" Handles automatic recharging of the turtlebots. """

def add_start_charging_action(node):
    node.actions.append(vda5050.Action("startCharging", "0", vda5050.BlockingType.SOFT))


def generate_stop_charging_action(agv):
    action_list = [vda5050.Action(
        action_type='stopCharging',
        action_id='0',
        blocking_type=vda5050.BlockingType.SOFT,
    )]
    vda5050_instant_action = vda5050.InstantAction(
        header_id=0,
        timestamp='',
        version=agv.version,
        manufacturer=agv.manufacturer,
        serial_number=str(agv.aid),
        instant_actions=action_list
    )
    mqtt.client.publish(vda5050.get_mqtt_topic(str(agv.aid), vda5050.Topic.INSTANT_ACTIONS),
                        vda5050_instant_action.json(), 2)


def generate_recharge_orders(graph):
    while True:
        for agv in graph.get_agvs():
            if agv.connection_status == "ONLINE":
                if agv.battery_level is not None and agv.battery_level < 20:
                    if agv.charging_status == "Discharging" and not agv.charging_prepare:
                        dist_to_charge_node = sys.maxsize
                        dest_nodes = list()
                        for node in graph.nodes:
                            if node.name is not None and "charge" in node.name:
                                agv_node = graph.get_nearest_node_from_agv(agv)
                                nodes, edges = graph.get_shortest_route(agv_node, node)
                                length = 0.0
                                for edge in edges:
                                    length += edge.length
                                if length < dist_to_charge_node:
                                    dist_to_charge_node = length
                                    dest_nodes = nodes
                        print("Generating recharging order for agv " + str(agv.aid) + " to " + dest_nodes[-1].name)
                        add_start_charging_action(dest_nodes[-1])
                        graph.append_new_order(dest_nodes[0].nid, dest_nodes[-1].nid, str(agv.aid), OrderType.RECHARGE)
                        agv.charging_prepare = True
                elif agv.battery_level is not None and agv.battery_level > 40:
                    if agv.charging_status == "Charging" and agv.charging_prepare:
                        print("Stop charging for agv " + str(agv.aid))
                        generate_stop_charging_action(agv)
                        agv.charging_prepare = False
        time.sleep(1)
