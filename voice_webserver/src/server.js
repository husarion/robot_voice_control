const https = require('https');
const socketIO = require('socket.io');
//const DeepSpeech = require('deepspeech-gpu');
const VAD = require('node-vad')
const fs = require('fs');
const express = require('express');
const Matcher = require('./server/dictionary-match.js')
const Ros = require('./server/ros.js');


const CONFIG = require('config').get("config");

const {Model}= CONFIG.GPU ? require('deepspeech-gpu') : require('deepspeech');


const topics = {
	orders: '/orders',
	feedback: '/orders_feedback',
	dictionary: 'txt_to_orders/dictionary',
	txt_to_orders: '/txt_to_orders/txt_to_orders',
	cancel_order: "/orders/cancel_one",
	cancel_all_orders: "/orders/cancel_all"
};


let DEEPSPEECH_MODEL = __dirname + '/deepspeech-0.6.0-models/'; // path to deepspeech english model directory
const vad = new VAD(VAD.Mode.VERY_AGGRESSIVE);

const certificate = {
	key: fs.readFileSync(`${__dirname}/cert/key.key`),
	cert: fs.readFileSync(`${__dirname}/cert/cert.crt`)
};

function createModel(modelDir, options) {
	let modelPath = modelDir + '/output_graph.pbmm';
	let lmPath = modelDir + '/lm.binary';
	let triePath = modelDir + '/trie';
	let model = new Model(modelPath, options.BEAM_WIDTH);
	model.enableDecoderWithLM(lmPath, triePath, options.LM_ALPHA, options.LM_BETA);
	return model;
}

let englishModel = createModel(DEEPSPEECH_MODEL, CONFIG);
//let matcher = new Matcher(dict);
let matcher = null;
let ros = new Ros(topics);

let modelStream;
let recordedChunks = 0;
let silenceStart = null;
let endTimeout = null;
let silenceBuffers = [];

function processAudioStream(data, callback) {
	vad.processAudio(data, 16000).then((res) => {
		switch (res) {
			case VAD.Event.ERROR:
				console.log("VAD ERROR");
				break;
			case VAD.Event.NOISE:
				console.log("VAD NOISE");
				break;
			case VAD.Event.SILENCE:
				processSilence(data, callback);
				break;
			case VAD.Event.VOICE:
				processVoice(data);
				break;
			default:
				console.log('default', res);
				
		}

	});
	
	// timeout after 1s of inactivity
	clearTimeout(endTimeout);
	endTimeout = setTimeout(function() {
		console.log('timeout');
		resetAudioStream();
	}, 1000);
}

function endAudioStream(callback) {
	console.log('[end]');
	let text = intermediateDecode();
	if (text) {
		if (callback) {
			callback(text);
		}
	}
}

function resetAudioStream() {
	clearTimeout(endTimeout);
	console.log('[reset]');
	recordedChunks = 0;
	silenceStart = null;
}

function processSilence(data, callback) {
	if (recordedChunks > 0) { // recording is on
		process.stdout.write('-'); // silence detected while recording
		
		feedAudioContent(data);
		
		if (silenceStart === null) {
			silenceStart = new Date().getTime();
		}
		else {
			let now = new Date().getTime();
			if (now - silenceStart > CONFIG.SILENCE_THRESHOLD) {
				silenceStart = null;
				console.log('[end]');
				let text = intermediateDecode();
				if (text) {
					if (callback) {
						callback(text);
					}
				}
			}
		}
	}
	else {
		process.stdout.write('.'); // silence detected while not recording
		bufferSilence(data);
	}
}

function bufferSilence(data) {
	// VAD has a tendency to cut the beggining of audio data from the start of a recording
	// so keep a buffer of audio and in addBufferedSilence() reattach it to the beginning of the recording
	silenceBuffers.push(data);
	if (silenceBuffers.length >= 16) {
		silenceBuffers.shift();
	}
}

function addBufferedSilence(data) {
	let audioBuffer;
	if (silenceBuffers.length) {
		silenceBuffers.push(data);
		let length = 0;
		silenceBuffers.forEach(function (buf) {
			length += buf.length;
		});
		audioBuffer = Buffer.concat(silenceBuffers, length);
		silenceBuffers = [];
	}
	else audioBuffer = data;
	return audioBuffer;
}

function processVoice(data) {
	silenceStart = null;
	if (recordedChunks === 0) {
		console.log('');
		process.stdout.write('[start]'); // recording started
	}
	else {
		process.stdout.write('='); // still recording
	}
	recordedChunks++;
	
	data = addBufferedSilence(data);
	feedAudioContent(data);
}

function createStream() {
	modelStream = englishModel.createStream();
	recordedChunks = 0;
}

function finishStream() {
	if (modelStream) {
		let text = englishModel.finishStream(modelStream);
		if (text.length > 1) {
			//silent can result in singular letters due to bug in this model
			console.log('');
			console.log('New text:', text);
			text = matcher.matchTextToDictionary(text);
			return text;
		}
			
	}
	modelStream = null;
	silenceBuffers = [];
	return;
}

function intermediateDecode() {
	let text = finishStream();
	createStream();
	return text;
}

function feedAudioContent(chunk) {
	englishModel.feedAudioContent(modelStream, chunk.slice(0, chunk.length/2));
}

///////////////////////////////////////////////////////////////////////////////////

const app = express();

const server = https.createServer(certificate, app);

const io = socketIO(server, {});

io.set('origins', '*:*');

io.on('connection', (socket) => {
	const emitRecognizedTxt = (text) => {
		if (text) {
			socket.emit('recognize', text); //pure text - mainly for debug purposes
			ros.txt_to_orders(text, socket, "orders"); //txt represented as orders
		}
	};
	
	console.log('client connected');
	
	socket.once('disconnect', () => {
		console.log('client disconnected');
	});
	
	createStream();
	
	socket.on('stream-data', (data) => {
		processAudioStream(data, emitRecognizedTxt)
	});
	
	//stream-end happens when user is silent
	//We publish msg on socket to show intermediate results to user 
	socket.on('stream-end', () => {
		console.log('end audio stream invoked');
		endAudioStream(emitRecognizedTxt);
	});
	
	//it is received whenever user stop recoding
	socket.on('stream-finish', () => {
		endAudioStream(emitRecognizedTxt);
		resetAudioStream();
	});
	
	socket.on('orders', (data) => {
		ros.pub_orders(data);
	});

	socket.on('cancel', (req) =>{
		(req === "all") ? ros.cancel_all_orders() : ros.cancel_order(req)
	});
});

server.listen(CONFIG.SOCKET_PORT, CONFIG.HOSTNAME, () => {
	console.log('Socket server listening on:', CONFIG.SOCKET_PORT);
});

ros.connect();

ros.on('feedback', (msg) => {
	io.emit('feedback', msg);
});

ros.once('dictionary', (msg) => { 
	matcher = new Matcher(msg.data)
	console.log("got dictionary")
	ros.unsubscribe(topics.dictionary);
});


module.exports = app;
