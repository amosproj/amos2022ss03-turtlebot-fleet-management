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

        }
    },
    methods: {
        refreshMap() {
            document.getElementById('graphmap').src = 'graph?' + Math.random()
        },
        async sendOrder() {
            await axios.post('/api/agv/' + this.robotSerial + '/sendFromTo/' + this.fromStation + '/' + this.toStation)
        },
        async updateUIdata() {
            // This is kind of a quick and dirty function, refreshing everything every second
            // An update like this is better done via WebSockets

            this.refreshMap()

            const orders_promise = axios.get('/api/orders')
            const agv_states_promise = axios.get('/api/agv/info')
            const graph_node_promise = axios.get('/api/graph/coordinates')

            this.orders = (await orders_promise).data
            this.agvs = (await agv_states_promise).data
              this.graph = (await graph_node_promise).data


            window.myLineChart = new Chart(document.getElementById("myChart"), {
                type: 'scatter',
                data: {
                datasets: [{
                label: "Test-Graph",
                fill: false,
                borderColor: "green",
                data: this.graph[1] ,
                        }]
                         },
                options: {
                       response : true,
                       legend: {
                            display: false
                             },
                       scales: {
                        xAxes: [{
                            gridLines: {
                                drawOnChartArea: false
                        }
                      }],
                        yAxes: [{
                            gridLines: {
                                drawOnChartArea: false
                        }
                      }]
                    }
                        }
                    });
            for(let i=0;i<this.graph.length;i++){
                     myLineChart.data.datasets.push({
                            label: "item "+i,
                            fill: false,
                            borderColor: "green",
                            data: this.graph[i],
                        });
                 }
            window.myLineChart.update();
                        }
    },
    created() {
        setInterval(this.updateUIdata, 4000)
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
