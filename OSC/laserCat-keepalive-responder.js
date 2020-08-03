module.exports = {
    oscInFilter:function(data){
        var {address, args, host, port} = data
        if (address === '/laserCat/keepAlive') {
            send(host, port, '/laserCat/keepAlive', args[0].value)
            //return // bypass original message
        }
        return {address, args, host, port}
    },
}