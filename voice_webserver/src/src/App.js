import React, {Component} from 'react';
import io from 'socket.io-client';

import {ClickableList, Sortable} from './Containers.js'
import FeedbackTable from './Table.js'

const DOWNSAMPLING_WORKER = './downsampling_worker.js';

class App extends Component {
	constructor(props) {
		super(props);
		this.state = {
			connected: false,
			recording: false,
			recordingStart: 0,
			recordingTime: 0,
			recognitionOutput: "",
			orders: [],
			errors: [],
			current_order: null,
			next_orders: [],
			history: [],
			maxHistory: 10,
			last_noticed_id : null
		};
		
	}
	
	componentDidMount = () => {
		let hostname = document.URL.split("//").pop().split(":")[0];
		this.socket = io.connect(`https://${hostname}:4000`, 
						{rejectUnauthorized: false, reconnect: true});

		this.socket.on('connect', () => {
			console.log('socket connected');
			this.setState({connected: true});
		});

		this.socket.on('disconnect', () => {
			console.log('socket disconnected');
			this.setState({connected: false});
			this.stopRecording();
		});

		this.socket.on('recognize', (results) => {
			console.log('recognized:', results);
			this.setState({recognitionOutput: `${this.state.recognitionOutput} ${results}`}); //extend current output by results
		});

		this.socket.on('orders', (orders) =>{
			this.setState({orders: this.state.orders.concat(orders.orders.data),
							errors: orders.errors
			});
		});

		this.socket.on('feedback', (feedback) =>{ 
			this.setState({current_order: feedback.current_order,
							next_orders: feedback.next_orders});
			this.updateHistory(feedback.previous_order);
		});
	}

	render() {
		return (<div className="App">
			<div>
				{this.renderRecordButtons()}	
				{this.renderOrders()} 
				{this.renderCurrOrderFB()}
				{this.renderNextOrdersFB()}
				{this.renderHistory()}
			</div>
		</div>); 
	}

	renderRecordButtons(){
		return( 
			<div>
				<button disabled={!this.state.connected || this.state.recording} onClick={this.startRecording}>
					Start Recording
				</button>

				<button disabled={!this.state.recording} onClick={this.stopRecording}>
					Stop Recording
				</button>
	    	</div>
		);
	}

	renderOrders (){
		return( 
			<div>
				<Sortable  
					elements={this.state.orders} 
					onDoubleClick={this.clearOrder}
					sequenceChanged={(orders) => this.setState({orders: orders})} 
					show="description"
				/>	

				<button hidden={!this.state.orders.length}
						onClick={this.sendOrders}>
					Send Orders
				</button>

				<button hidden={!this.state.orders.length}
						onClick={() => this.setState({orders: []})}>
					Clear Orders
				</button>

			</div>
		);
	}

	renderCurrOrderFB(){
		return (
			<div className="currentOrder">
				<FeedbackTable title="Current order" 
					cols={["description", "distance_linear", "distance_angular", "progress"]}
					data={[this.state.current_order]} />
				
				<button disabled={this.state.current_order === null || this.state.current_order.description === ""} 
					onClick={() => this.cancelSentOrder(this.state.current_order)}
				>
					Cancel current
				</button>
			</div>
		); 
	}


	renderHistory(){
		return (
			<div className="history">
				<FeedbackTable title="History" 
					cols={["description", "result", "success"]}
					data={this.state.history} 
				/>	

				<button disabled={!this.state.history.length}
					onClick={() => this.setState({history: []})}>
					Clear History
				</button>
			</div>
	); 
	}

	updateHistory(previous_order) {
		let history = this.state.history
		if (previous_order.description !== "" && (previous_order.id !== this.state.last_noticed_id)){
			history.unshift(previous_order);
			history = history.slice(0, this.state.maxHistory);
			this.setState({history: history, last_noticed_id : previous_order.id});
		}
	}

	renderNextOrdersFB(){
		return (
			<div className="nextOrders">
				<ClickableList 
					title="Next Orders"
					elements={this.state.next_orders} 
					onDoubleClick={(idx, order) => this.cancelSentOrder(order)}
					show="description"
				/>
				<button disabled={!this.state.next_orders.length} onClick={this.cancelAllSent}>
					Cancel All
				</button>
			</div>
		); 
	}

	cancelSentOrder = (order) =>{
		let confirmed = window.confirm(`are you sure want to cancel order: ${order.description}?`)
		if (confirmed)
			this.socket.emit('cancel', order.id);
	} 

	cancelAllSent = () => {
		let confirmed = window.confirm(`are you sure want to cancel all orders?. It will also cancel current order`)
		if (confirmed)
			this.socket.emit('cancel', 'all');
	}

	clearOrder = (idx) => {
		let orders = this.state.orders;
		orders.splice(idx, 1);
		this.setState({orders: orders})
	}	

	createAudioProcessor(audioContext, audioSource) {
		let processor = audioContext.createScriptProcessor(4096, 1, 1);
		
		const sampleRate = audioSource.context.sampleRate;
		
		let downsampler = new Worker(DOWNSAMPLING_WORKER);
		downsampler.postMessage({command: "init", inputSampleRate: sampleRate});
		downsampler.onmessage = (e) => {
			if (this.socket.connected) {
				this.socket.emit('stream-data', e.data.buffer);
			}
		};
		
		processor.onaudioprocess = (event) => {
			var data = event.inputBuffer.getChannelData(0);
			downsampler.postMessage({command: "process", inputFrame: data});
		};
		
		processor.shutdown = () => {
			processor.disconnect();
			this.onaudioprocess = null;
		};
		
		processor.connect(audioContext.destination);
		
		return processor;
	}

	sendOrders = e => {
		this.socket.emit("orders", this.state.orders)
		this.setState({recognitionOutput: "",
					   orders: []
		});
	}

	startRecording = e => {
		if (!this.state.recording) {
			this.recordingInterval = setInterval(() => {
				let recordingTime = new Date().getTime() - this.state.recordingStart;
				this.setState({recordingTime});
			}, 100);
			
			this.setState({
				recording: true,
				recordingStart: new Date().getTime(),
				recordingTime: 0
			}, () => {
				this.startMicrophone();
			});
		}
	};
	
	startMicrophone() {
		this.audioContext = new AudioContext();
		
		const success = (stream) => {
			console.log('started recording');
			this.mediaStream = stream;
			this.mediaStreamSource = this.audioContext.createMediaStreamSource(stream);
			this.processor = this.createAudioProcessor(this.audioContext, this.mediaStreamSource);
			this.mediaStreamSource.connect(this.processor);
		};
		
		const fail = (e) => {
			console.error('recording failure', e);
		};
		
		if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
			navigator.mediaDevices.getUserMedia({
				video: false,
				audio: true
			})
			.then(success)
			.catch(fail);
		}
		else {
			navigator.getUserMedia({
				video: false,
				audio: true
			}, success, fail);
		}
	}
	
	stopRecording = e => {
		if (this.state.recording) {	
			if (this.socket.connected) {
				this.socket.emit('stream-finish');
			}

			clearInterval(this.recordingInterval);
			this.setState({
				recording: false,
			}, () => {
				this.stopMicrophone();
			});
		
		}
	};
	
	stopMicrophone() {
		if (this.mediaStream) {
			this.mediaStream.getTracks()[0].stop();
		}
		if (this.mediaStreamSource) {
			this.mediaStreamSource.disconnect();
		}
		if (this.processor) {
			this.processor.shutdown();
		}
		if (this.audioContext) {
			this.audioContext.close();
		}
	}

}

export default App;
