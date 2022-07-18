const {createApp} = Vue
var points = new Array()
const chartjs_config = {
                type: 'scatter',
                data: {
                datasets: [{
                label: "Test-Graph",
                fill: false,
                borderColor: "gray",
                showLine: true,
                data: null,
                        }]
                         },
                options: {
                              animation: {
        duration: 0, // general animation time
    },
                          plugins:{
                         autocolors: false,
                          annotation: {
                            annotations: points
                          },
                       legend: {
                            display: false
                             }
                              },
                       scales: {
                        x: {
                            grid: {
                                display: false
                        }
                      },
                        y: {
                            grid: {
                                display: false
                        }
                      }
                    }
                       },
                    plugins: ['chartjs-plugin-annotation']
                    }

createApp({
    data() {
        return {
            pathUrl: null,
            stations: [],
            robotSerial: "1",
            fromStation: null,
            toStation: null,
            orders: [],
            agvs: [],
            graph: [],
            graph_edges: [],



        }
    },
    methods: {
        refreshMap() {
           document.getElementById('graphmap').src = 'graph?' + Math.random()
        },
        async graphDataSetting(){
            const graph_data_promise = axios.get('/api/graph')
            const graph_data_edges = axios.get('/api/graph/edges')
            const agv_states_promise = axios.get('/api/agv')


            this.agvs = (await agv_states_promise).data
            this.graph = (await graph_data_promise).data
            this.graph_edges = (await graph_data_edges).data

            var graph_nodes = this.graph.nodes

            for(let i=0;i<this.graph_edges.length;i++){
                     myLineChart.data.datasets.push({
                        label: 'Edge_'+ i ,
                        data: this.graph_edges[i],
                        fill: false,
                        showLine: true,
                        borderColor: 'gray',
                        tension: 0.1,
                        order: 1
                        });
                 };
            for(let i=0;i<graph_nodes.length;i++){
                if (graph_nodes[i].name != null ){
                     points.push({
                          type: 'point',
                          backgroundColor: 'red',
                          borderColor: 'blue',
                          borderWidth: 1,
                          pointStyle: 'rectRot',
                          scaleID: 'y',
                          xValue: graph_nodes[i].x,
                          yValue: graph_nodes[i].y
                        });
                 }};
                 for(let i=0;i<graph_nodes.length;i++){
                 if (graph_nodes[i].name != null ){
                     points.push({
                          type: 'label',
                          borderColor: 'green',
                          borderRadius: 7,
                          borderWidth: 2,
                          content: [graph_nodes[i].name+' ('+graph_nodes[i].nid+')'],
                          font: {
                            size:16
                          },
                          position: {
                            x: 'end',
                            y: 'end'
                          },
                          xValue: graph_nodes[i].x,
                          yValue: graph_nodes[i].y
                        });
                                      }};

                 for(let i=0;i<this.agvs.length;i++){
                     points.push({
                          type: 'point',
                          backgroundColor: this.agvs[i].color,
                          borderColor: 'blue',
                          borderWidth: 1,
                          pointStyle: 'rectRot',
                          scaleID: 'y',
                          xValue:  this.agvs[i].x,
                          yValue:  this.agvs[i].y
                        });
                 };
                 myLineChart.update();

        },
        highlightPathOrderHorizon(graph_orders,order_nodes) {

                    var Horizons_orders= new Array()
            for (let i=0;i<graph_orders.length;i++){
            Horizons_orders.push({"hor":graph_orders[i].horizon,"index":graph_orders[i].agv})
            }


            var Horizon_coordinate = new Array()

            for (let k=0;k<Horizons_orders.length;k++){
            var Horizons = Horizons_orders[k].hor
            var colour_index  = Horizons_orders[k].index
            var color = this.agvs[parseInt(colour_index)-1].color
            var temp_horizon = new Array()
            for (let i=0;i<Horizons.length;i++){
              for (let j=0;j<order_nodes.length;j++){
              if (Horizons[i] == order_nodes[j].nid){
              temp_horizon.push({"x":order_nodes[j].x,"y":order_nodes[j].y})
              }
              }

            }Horizon_coordinate.push({"horico":temp_horizon,"color":color})

            };
                 for (let i=0;i<Horizon_coordinate.length;i++){
                     myLineChart.data.datasets.push({
                        label: 'orders_1',
                        data: Horizon_coordinate[i].horico,
                        fill: false,
                        showLine: true,
                        borderColor: Horizon_coordinate[i].color,
                        tension: 0.1,
                        order: 0
                        });
            }


            myLineChart.update();


        },
        highlightPathOrderBase(graph_orders,order_nodes) {

                    var Base_orders= new Array()
            for (let i=0;i<graph_orders.length;i++){
            Base_orders.push({"base":graph_orders[i].horizon,"index":graph_orders[i].agv})
            }


            var Base_coordinate = new Array()

            for (let k=0;k<Base_orders.length;k++){
            var Bases = Base_orders[k].base
            var colour_index  = Base_orders[k].index
            var color = this.agvs[parseInt(colour_index)-1].color
            var temp_base = new Array()
            for (let i=0;i<Bases.length;i++){
              for (let j=0;j<order_nodes.length;j++){
              if (Bases[i] == order_nodes[j].nid){
              temp_base.push({"x":order_nodes[j].x,"y":order_nodes[j].y})
              }
              }

            }Base_coordinate.push({"baseco":temp_base,"color":color})

            };
                 for (let i=0;i<Base_coordinate.length;i++){
                     myLineChart.data.datasets.push({
                        label: 'orders_1',
                        data: Base_coordinate[i].baseco,
                        fill: false,
                        showLine: true,
                        borderColor: "green",
                        tension: 0.1,
                        order: 0
                        });
            }


            myLineChart.update();


        },
        async sendOrder() {
            await axios.post('/api/agv/' + this.robotSerial + '/sendFromTo/' + this.fromStation + '/' + this.toStation)
        },
        async cancelOrder(id) {
            await axios.delete('/api/orders/' + id)
        },
        async resendOrder(id) {
            await axios.post('/api/orders/' + id + '/resend')
        },
        async updateUIdata() {
            // This is kind of a quick and dirty function, refreshing everything every second
            // An update like this is better done via WebSockets

            this.refreshMap()
            const agv_states_promise = axios.get('/api/agv')
            const graph_data_promise = axios.get('/api/graph')
            const order_data_promise = axios.get('/api/orders')

            this.agvs = (await agv_states_promise).data
            this.graph = (await graph_data_promise).data
            this.orders = (await order_data_promise).data

            for (let i=0;i<this.agvs.length;i++){
            points[points.length-(this.agvs.length-i)].xValue = this.agvs[i].x
            points[points.length-(this.agvs.length-i)].yValue = this.agvs[i].y
            }

            var graph_orders = this.graph.orders
            var order_nodes = this.graph.nodes

            this.highlightPathOrderHorizon(graph_orders,order_nodes)
            this.highlightPathOrderBase(graph_orders,order_nodes)
        }

    },
    created() {

    },
    async mounted() {
        let result = await axios.get('/api/graph/stations')
        this.stations = result.data
        this.fromStation = this.stations[0].nid
        this.toStation = this.stations[1].nid
        myLineChart = new Chart('ChartJsChart',chartjs_config);
        this.graphDataSetting()
        setInterval(this.updateUIdata, 1000)

        /*
        const canvas = document.querySelector('#canvas');

        if (!canvas.getContext) {
            return;
        }
        const ctx = canvas.getContext('2d');

        axios.get('/graph.json')
                .then(function (response) {
                    const nodes = response.data.nodes
                    const edges = response.data.edges

                    ctx.strokeStyle = 'red';
                    ctx.lineWidth = 3;

                    for(const edge of edges) {
                        ctx.beginPath();
                        const start = nodes[edge.start]
                        const end = nodes[edge.end]
                        ctx.moveTo(start.x * 10, start.y * 10);
                        ctx.lineTo(end.x * 10, end.y * 10);
                        ctx.stroke();
                    }

            })
                .catch(function (error) {
                    console.log(error)
            })
        */
    }
}).mount('#app')
