import time
import sys
import mqtt
import vda5050


def generate_recharge_orders(graph):
    return
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
                        dest_nodes[-1].actions.append(vda5050.Action("startCharging", "0", "SOFT"))
                        graph.append_new_order(dest_nodes[0].nid, dest_nodes[-1].nid, str(agv.aid))
                        agv.charging_prepare = True
                elif agv.battery_level is not None and agv.battery_level > 27:
                    if agv.charging_status == "Charging" and agv.charging_prepare:
                        print("Stop charging for agv " + str(agv.aid))
                        action_list = []
                        action_list.append(vda5050.Action(
                            action_type='stopCharging',
                            action_id='0',
                            blocking_type=vda5050.BlockingType.SOFT,
                        ))
                        vda5050_instant_action = vda5050.InstantAction(
                           header_id=0,
                           timestamp='',
                           version='',
                           manufacturer='',
                           serialnumber=str(agv.aid),
                           instantActions=action_list
                        )
                        mqtt.client.publish(vda5050.get_mqtt_topic(str(agv.aid), vda5050.Topic.INSTANT_ACTIONS),
                                            vda5050_instant_action.json(), 2)
                        agv.charging_prepare = False

        time.sleep(1)

