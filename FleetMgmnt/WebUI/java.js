const {createApp} = Vue


createApp({
    data() {
        return {
            message: 'Hello Vue!',
            stationID: 1,
            robotSerial: "1",
            fromNode: "12",
            targetNode: "5",
            pathUrl: null,
            stations: [],
            fromStation: null,
            toStation: null,
            agvs: [],
            orders: [],
        }
    },
    methods: {
        refreshMap() {
            document.getElementById('graphmap').src = 'graph?' + Math.random()
        },
        sendReqToFleetManagement(endpoint) {
            axios.post('/api/station/' + this.stationID + '/' + endpoint)
                .then(function (response) {
                    console.log(response)
                })
                .catch(function (error) {
                    console.log(error)
                })
                .then(function () {
                });
        },
        requestAGV() {
            this.sendReqToFleetManagement('req')
        },
        moveAGV() {
            this.sendReqToFleetManagement('move')
        },
        sendAGVTo(station) {
            this.sendReqToFleetManagement('sendTo/' + station)
        },
        sendOrder() {
            axios.post('/api/agv/' + this.robotSerial + '/sendFromTo/' + this.fromStation + '/' + this.toStation)
                .then(function (response) {
                    console.log(response)
                })
                .catch(function (error) {
                    console.log(error)
                })
                .then(function () {
                });
        },
        async updateUIdata() {
            // This is kind of a quick and dirty function, refreshing everything every second
            // An update like this is better done via WebSockets

            this.refreshMap()

            const orders_promise = axios.get('/api/orders')
            const agv_states_promise = axios.get('/api/agv/info')

            this.orders = (await orders_promise).data
            this.agvs = (await agv_states_promise).data
        }
    },
    created() {
        setInterval(this.updateUIdata, 1000)
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
