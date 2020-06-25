import React, {Component} from 'react';


class Table extends Component {
    render(){
        return (
            <table>
                <thead>
                    {this.renderTitle()}
                    {this.renderHeader()}
                </thead>
                <tbody>
                    {this.renderData()}
                </tbody>
            </table> 
        );
    }

    renderTitle(){
        if (this.props.title){
            return(
                <tr key={this.props.title}>
                    <th colSpan={this.props.cols.length}>
                        {this.props.title}
                    </th>
                </tr>
            );
        }
    }


    renderHeader(){
        return (
                <tr key="header">
                    {this.props.cols.map((column) => 
                        (<th key={column}>{column}</th>)
                    )}
                </tr>
        );
    }

    renderData(){
        return (
            this.props.data.map((row, idx) =>
                (row !== null) ? <tr key={idx}>{this.renderRow(row)}</tr> : null)
        );
    }

    renderRow(row){
        return(
            this.props.cols.map((column, idx) => (
                <td key={idx}>{this.renderCell(row, column)}</td>
        
            ))
        );
        
    }

    renderCell(row, column){
        return row[column];
    }
}

export default class FeedbackTable extends Table{
    renderHeader(){
        return (
                <tr key="header">
                    {this.props.cols.map((column) => 
                        (<th key={column}>{column.replace("_", " ")}</th>)
                    )}
                </tr>
        );
    }

    renderCell(row, column){
        let cell = row[column]
        if (typeof(cell) == 'boolean')
        return this._renderBool(cell);
        
        else if(typeof(cell) == 'number'){
            return this._renderNum(cell);
        }
        else 
            return this._renderStr(cell);
    }   

    _renderBool(cell){
        return String(this._renderStr(cell));
    }
    
    _renderStr(cell){
        return cell;
    }

    _renderNum(cell){
        if (cell === 0)
            return null
        return cell.toFixed(3);
    }
}
