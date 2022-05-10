const { createApp } = Vue


createApp({
    data() {
        return {
            message: 'Hello Vue!',
            stationID: 1
        }
    },
    methods: {
        sendReqToFleetManagement(endpoint) {
            axios.post('/api/station/' + this.stationID + '/' + endpoint)
                .then(function (response) {
                    console.log(response);
                })
                .catch(function (error) {
                    console.log(error);
                })
                .then(function () {
                });
        },
        requestAGV() {this.sendReqToFleetManagement('req')},
        moveAGV() {this.sendReqToFleetManagement('move')},
        sendAGVTo(station) {this.sendReqToFleetManagement('sendTo/' + station)},
    }
}).mount('#app')
