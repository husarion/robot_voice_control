const metaphone = require('double-metaphone');
const levenshtein = require('levenshtein-edit-distance')

module.exports = class Matcher {
    constructor(dictionary)
    {
        [this.dictionary, this.dictKeys] = this._prepareDict(dictionary);
        
        //for description of below fields see ._isDistBelowThreshold function
        this._metaTh = { //used to determine whether phonetics of two words is similar
            th1: 1.5,
            th2: 2.5,
            step: 2
        }
        this._levenNoMatchTh = { //used to pick word if no metaphone match was found 
            th1: 1.5,
            th2: 2.5,
            step: 3
        }
        this._levenMatchTh = null; //used to pick word if more than one matches were found using metaphone
    }

    matchTextToDictionary(text){
        if (this.dictionary != null){
            let words = text.match(/[^ ]+/g)

            let newWords = [];
            words = words.forEach( (word) => { 
                let matched = this._matchWord(word)
                if (matched != null) newWords.push(matched); 
            });
            
            text = newWords.join(" ");
            //console.log(`metaphone out: ${text}`);
        }   

        return text; 
    }   

    _prepareDict(dictionary){
        let keysOnly = []
        let dict = []
        dictionary.forEach((word) => {
            dict.push([word, metaphone(word)]);
            keysOnly.push(word);
        });
        return [dict, keysOnly];
    }

    _matchWord(word){
        //first we try to find word from dictionary which phonetics is close to input word (metaphone+levenshtein distance),
        //then if it fails or if it finds many similar words we calculate levenshtein distance on pure words (without metaphone)
        //and pick one with smallest distance 

        //below returns word described as two metawords - primary and secondary
        //if two primary keys match each other, then similarity is strong
        let metawords = metaphone(word); 
        

        let metaCandidates = new Set(); //current candidates determined by meta description
        let metaDistance; //levenshtein distance between meta of word and current candidate
        // let candidate;
        // let cCharDistance; //candidate distances
        // let cMetaDistance;

        // for (const [key, metakey] of this.dictionary) {
        //     if (word == key) return key;
        //     for (let i=0; i<2; i++) {
        //         let metaword = metawords[i];
        //         //it can happen that word has only one meta representation, thus we check
        //         if (!metaword) continue;
                
        //         for (let j=0; j<2; j++) {
        //             let mkey = metakey[j];
        //             if (!mkey) continue;
        //             //bool in this equation will penalize if secondary keys are matched
        //             let chardist = levenshtein(word, key);
        //             let metadist = levenshtein(metaword, mkey) 
        //             let dist = metadist + chardist;
        //             let metadistOK = true;
        //             //if (key == "reach" || key == "right") console.log(metadist, key);
        //             //let metadistOK = this._isDistBelowThreshold(metadist, metaword, mkey, this._metaTh);
                    
        //             let candidateDist = cCharDistance + cMetaDistance;
        //             if ((candidate == null || dist < candidateDist) || (dist == candidateDist && metadist < cMetaDistance)
        //                 || (dist == candidateDist && metadist == cMetaDistance && chardist < cCharDistance))
        //             {
        //                 candidate = key;
        //                 cCharDistance = chardist;
        //                 cMetaDistance = metadist; 
        //             }
        //         }
        //    }
        // }
        for (const [key, metakey] of this.dictionary) {
            if (word == key) return key;
            for (let i=0; i<2; i++) {
                let metaword = metawords[i];
                //it can happen that word has only one meta representation, thus we check
                if (!metaword) continue;
                
                for (let j=0; j<2; j++) {
                    let mkey = metakey[j];
                    if (!mkey) continue;
                    //bool in this equation will penalize if secondary keys are matched
                    let metadist = levenshtein(metaword, mkey) + 0.5*Boolean(Math.max(i, j));
                    let metadistOK = this._isDistBelowThreshold(metadist, metaword, mkey, this._metaTh);
                        
                    if (metadist == metaDistance && metadistOK) {
                        metaCandidates.add(key);
                    }
                    else if ((metaDistance == null || metadist < metaDistance) && metadistOK){
                        metaCandidates.clear()
                        metaCandidates.add(key);
                        metaDistance = metadist;
                    }
                }
            }
        }
         switch (metaCandidates.size){
            case 0:
                return this._levenCandidate(word, this.dictKeys, this._levenNoMatchTh);
            case 1:
                return metaCandidates.values().next().value;
            default:
                return this._levenCandidate(word, metaCandidates, this._levenMatchTh);
        };
    }
            

    _isDistBelowThreshold(dist, word1, word2, thresh){
        //value of this statement is determined considering two cases - long and short strings
        //thresh.step determines max number of characters that string can have to consider it small 
        //thresh.th1 is threshold of distance for small strings, thresh.th2 for bigger
        if (thresh == null) return true;
        
        let length = Math.max(word1.length, word2.length);
        if ((length <= thresh.step && dist <= thresh.th1) || (length > thresh.step && dist <= thresh.th2 )){
            return true;
        }
        return false;
    }
    _levenCandidate(word, currentCandidates, levenTh){
        let levenDistance; //current smallest distance
        let levenCandidate; //current candidate
        
        currentCandidates.forEach( (candidate) => {
            let levendist = levenshtein(candidate, word);
            let levendistOK = this._isDistBelowThreshold(levendist, word, candidate, levenTh);
            if ((levenDistance == null || levendist < levenDistance) && levendistOK){
                levenCandidate = candidate;
                levenDistance = levendist;
            }
        });
        return levenCandidate;
    }
}