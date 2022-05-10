const { createApp } = Vue


createApp({
    data() {
        return {
            message: 'Hello Vue!'
        }
    },
}).mount('#app')

axios.get('/hans')
    .then(function (response) {
        // handle success
        console.log(response);
    })
    .catch(function (error) {
        // handle error
        console.log(error);
    })
    .then(function () {
        // always executed
    });