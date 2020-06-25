import React, {Component} from 'react';
import {ReactSortable} from 'react-sortablejs'

export class ClickableList extends Component{


    render(){
        return (
            <div>
                {this.renderTitle()}
                <ul>
                    {this.props.elements.map( (el, idx) => {
                        return this.renderElement(el, idx);
                    })} 
                </ul>
            </div>
        );
    }

    renderTitle(){
        return(
            <legend><b>{this.props.title}</b></legend>
        );
    }

    renderElement(el, id){
        return (
            <li key={id} draggable={true} onDoubleClick={() => this.props.onDoubleClick(id, el)}> 
                 {el[this.props.show]}
            </li>
        );
    }

}


export class Sortable extends ClickableList{
    render(){
        return (
            <div>
                {this.renderTitle()}
                <ReactSortable tag="ul" list={this.props.elements} setList={(newList) => this.props.sequenceChanged(newList)}>
                    {this.props.elements.map( (el, idx) => {
                        return this.renderElement(el, idx);
                    })} 
                </ReactSortable>
            </div>
        );
    }
}
    