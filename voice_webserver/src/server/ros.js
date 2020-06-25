const rosnodejs = require('rosnodejs')
const OSMsgs = rosnodejs.require('orders_supervisor').msg
const OSSrvs = rosnodejs.require('orders_supervisor').srv
const TxtToOrdersSrv = rosnodejs.require('txt_to_orders').srv.TxtToOrdersSrv
const DictionaryMsg = rosnodejs.require('txt_to_orders').msg.Dictionary
const Trigger = rosnodejs.require('std_srvs').srv.Trigger
const EventEmitter = require('events').EventEmitter

module.exports = class Ros {
    constructor(topics){
        this._nh  = null;
        this._topics = topics;
        this._orders_p = null;
        this._txt_to_orders_srv = null;
        this.emiter = new EventEmitter();
    }
    
    connect(){
        rosnodejs.initNode('webserver_node')
        this._nh  = rosnodejs.nh;
        this._orders_p = this._nh.advertise(this._topics.orders, OSMsgs.Orders);
            
        this._nh.subscribe(this._topics.feedback, OSMsgs.OrdersFeedback, this._feedback_cb);

        this._nh.subscribe(this._topics.dictionary, DictionaryMsg, this._dictionary_cb);
        
        this._nh.waitForService(this._topics.txt_to_orders, 3000)
        .then((avl) =>{
            if (!avl) console.log("\nWARNING!!! txt_to_orders srv not available \n ");
        })

        this._txt_to_orders_srv = this._nh.serviceClient(this._topics.txt_to_orders, TxtToOrdersSrv);
        this._cancel_order_srv = this._nh.serviceClient(this._topics.cancel_order, OSSrvs.CancelOrder);
        this._cancel_all_srv = this._nh.serviceClient(this._topics.cancel_all_orders, Trigger);

    }

    unsubscribe(topic_name){
        this._nh.unsubscribe(topic_name);
    }

    on(name, fn){
        this.emiter.on(name, fn);
    }

    once(name, fn){
        this.emiter.once(name, fn);
    }
    
    pub_orders = (data) => {
        let orders = new OSMsgs.Orders();
        orders.data = data;
        this._orders_p.publish(orders);
    }

    cancel_order = (idx) => {
        let cancelReq = new OSSrvs.CancelOrder.Request()
        cancelReq.id = idx;
        this._cancel_order_srv.call(cancelReq);
    }
    
    cancel_all_orders = () =>{
        this._cancel_all_srv.call(new Trigger.Request());
    }

    txt_to_orders = (txt, destination, topic) => {
        let request = new TxtToOrdersSrv.Request()
        request.text = txt;
        
        this._txt_to_orders_srv.call(request)
        .then((response) => {
            destination.emit(topic, response);
        })
        .catch((err) =>{ 
            console.log(err);
        });
    }
    
    _feedback_cb = (msg) =>{
        this.emiter.emit('feedback', msg);
    }
    _dictionary_cb = (msg) => {
        this.emiter.emit('dictionary', msg);
    }

    
}