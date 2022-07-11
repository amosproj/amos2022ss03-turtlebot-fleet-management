const {createApp} = Vue


createApp({
    data() {
        return {
            pathUrl: null,
            stations: [],
            robotSerial: "1",
            fromStation: null,
            toStation: null,
            agvs: [],
            orders: [],
            graph : [],
            orderpath : [],
            agv_pos_color : [],

        }
    },
    methods: {
        refreshMap() {
            document.getElementById('graphmap').src = 'graph?' + Math.random()
        },
        async sendOrder() {
            await axios.post('/api/agv/' + this.robotSerial + '/sendFromTo/' + this.fromStation + '/' + this.toStation)
            const graph_node_order_promise = axios.get('/api/agv/' + this.robotSerial + '/coordinate_pathDisplay/' + this.fromStation + '/' + this.toStation)
            this.orderpath = (await graph_node_order_promise).data
                        for(let i=0;i<this.orderpath.length;i++){
                     myLineChart.data.datasets.push({
                        label: 'Order_'+ i ,
                        data: this.orderpath[i],
                        fill: false,
                        showLine: true,
                        borderColor: 'red',
                        tension: 0.1,
                        order: 0
                        });
                 };
                 window.myLineChart.update();
        },
        async updateUIdata() {
            // This is kind of a quick and dirty function, refreshing everything every second
            // An update like this is better done via WebSockets

            this.refreshMap()

            const orders_promise = axios.get('/api/orders')
            const agv_states_promise = axios.get('/api/agv/info')
            const graph_node_promise = axios.get('/api/graph/coordinates')
            const agv_coordinate_color = axios.get('/api/graph/agv_coordinates')

            this.orders = (await orders_promise).data
            this.agvs = (await agv_states_promise).data
            this.graph = (await graph_node_promise).data
            this.agv_pos_color = (await agv_coordinate_color).data

            console.log(this.agv_pos_color)

            const points = new Array()
            window.myLineChart = new Chart('ChartJsChart', {
                type: 'scatter',
                data: {
                datasets: [{
                label: "Test-Graph",
                fill: false,
                borderColor: "gray",
                showLine: false,
                data: this.graph[0] ,
                        }]
                         },
                options: {
                          plugins:{
                         autocolors: false,
                          annotation: {
                            annotations:points
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
                    });
            for(let i=0;i<this.graph.length;i++){
                     myLineChart.data.datasets.push({
                        label: 'Edge_'+ i ,
                        data: this.graph[i],
                        fill: false,
                        showLine: true,
                        borderColor: 'gray',
                        tension: 0.1,
                        order: 1
                        });
                 };
            for(let i=0;i<this.stations.length;i++){
                     points.push({
                          type: 'point',
                          backgroundColor: 'red',
                          borderColor: 'blue',
                          borderWidth: 1,
                          pointStyle: 'rectRot',
                          scaleID: 'y',
                          xValue: this.stations[i].x,
                          yValue: this.stations[i].y
                        });
                 };
                 for(let i=0;i<this.stations.length;i++){
                     points.push({
                          type: 'label',
                          borderColor: 'green',
                          borderRadius: 7,
                          borderWidth: 2,
                          content: [this.stations[i].name+' ('+this.stations[i].nid+')'],
                          font: {
                            size:16
                          },
                          position: {
                            x: 'end',
                            y: 'end'
                          },
                          xValue: this.stations[i].x,
                          yValue: this.stations[i].y
                        });
                 for(let i=0;i<this.agv_pos_color.length;i++){
                     points.push({
                          type: 'point',
                          backgroundColor: this.agv_pos_color[i].color,
                          borderColor: 'blue',
                          borderWidth: 1,
                          pointStyle: 'rectRot',
                          scaleID: 'y',
                          xValue: this.agv_pos_color[i].x,
                          yValue: this.agv_pos_color[i].y
                        });
                 };

                 };
            window.myLineChart.update();


        }
    },
    created() {
        setInterval(this.updateUIdata, 2000)
    },
    async mounted() {
        let result = await axios.get('/api/graph/stations')
        this.stations = result.data
        this.fromStation = this.stations[0].nid
        this.toStation = this.stations[1].nid


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
