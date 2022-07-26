import sys

from matplotlib import pyplot as plt

from models.Order import Order, OrderStatus

""" This was just just for testing and visualizing the critical path calculation. """


def plot(graph):
    # drawing the edeges and saving fig to png takes most of the time
    fig1, ax1 = plt.subplots()

    order1 = Order(graph, graph.find_node_by_id(139), graph.find_node_by_id(128))
    order2 = Order(graph, graph.find_node_by_id(119), graph.find_node_by_id(61))
    graph.all_orders.append(order1)
    graph.all_orders.append(order2)
    order1.status = OrderStatus.ACTIVE
    order2.status = OrderStatus.ACTIVE

    for edge in graph.edges:
        ax1.plot(
            [edge.start.x, edge.end.x],
            [edge.start.y, edge.end.y],
            color="gray"
        )
        ax1.plot(
            edge.start.x, edge.start.y,
            marker='.',
            color="gray"
        )
        ax1.plot(
            edge.end.x, edge.end.y,
            marker='.',
            color="gray"
        )

    for i in range(60):
        order1.try_extension(0, 0)
    for i in range(30):
        print(order2.try_extension(0, 0))

    print(order1.base)
    # print(order1.horizon)
    print(order2.base)
    # print(order2.horizon)

    crit_nodes, _ = graph.order_critical_path_membership(order1)

    x, y = order1.get_cosp().exterior.xy
    ax1.plot(x, y, color='blue')

    x, y = order2.get_cosp().exterior.xy
    ax1.plot(x, y, color='orange')

    for node in graph.nodes:
        if node.lock == order1.order_id:
            ax1.plot(
                node.x, node.y,
                marker='D',
                color="blue"
            )
        if node.lock == order2.order_id:
            ax1.plot(
                node.x, node.y,
                marker='D',
                color="orange"
            )

    for node in crit_nodes:
        ax1.plot(
            node.x, node.y,
            marker='.',
            color="red"
        )

    for node in graph.nodes:
        if node.nid == 84:
            ax1.annotate(str(node.nid), (node.x, node.y))

    # start1 = graph.find_node_by_id(139)
    # end1 = graph.find_node_by_id(128)
    # start2 = graph.find_node_by_id(119)
    # end2 = graph.find_node_by_id(61)
    #
    # route1, _ = graph.get_shortest_route(start1, end1)
    # route2, _ = graph.get_shortest_route(start2, end2)
    #
    # osp1 = collavoid.get_path_safety_buffer_polygon((start1.x, start1.y), route1)
    # osp2 = collavoid.get_path_safety_buffer_polygon((start2.x, start2.y), route2)
    # locklist1 = graph.find_nodes_for_colocking(osp1)
    # locklist2 = graph.find_nodes_for_colocking(osp2)
    #
    # osp1_extended = osp1
    # for node in locklist1:
    #     osp1_extended = osp1_extended.union(node.buffer)
    #
    # osp2_extended = osp2
    # for node in locklist2:
    #     osp2_extended = osp2_extended.union(node.buffer)
    #
    # x, y = osp1_extended.exterior.xy
    # ax1.plot(x, y, color='red')
    # x, y = osp2_extended.exterior.xy
    # ax1.plot(x, y, color='blue')
    #
    # for node in locklist2:
    #     ax1.plot(
    #         node.x, node.y,
    #         marker='.',
    #         color="blue"
    #     )
    #
    # for node in locklist1:
    #     ax1.plot(
    #         node.x, node.y,
    #         marker='.',
    #         color="red"
    #     )
    #
    # critical_path = osp1_extended.intersection(osp2_extended)
    #
    # x, y = critical_path.exterior.xy
    # ax1.plot(x, y, color='orange')
    #
    # for node in graph.nodes:
    #     if critical_path.contains(node.spoint):
    #         ax1.plot(
    #             node.x, node.y,
    #             marker='.',
    #             color="green"
    #         )

    fig1.savefig("shapeltest.png", format="png", dpi=300, bbox_inches='tight')
    print("Done")
    sys.exit(0)


def haupt(graph):
    plot(graph)
