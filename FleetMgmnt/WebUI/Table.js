var table = null;
var FinalTable1 = [];
var Si_No=1;
function table_update(fromstation,tostation,serial,id){



	    var new_row = {}
	    new_row.SerialNo = Si_No
	    new_row.ID = id
	    new_row.From = fromstation
	    new_row.To = tostation
	    new_row.Status = "ACTIVE"
	    new_row.AGV = serial

		FinalTable1.push(new_row);

if(table == null)
var table = new Tabulator("#example-table", {
  data:FinalTable1, //assign data to table
    autoColumns:true, //create columns from data field names
     layout:"fitDataFill",
      resizableRows:true,
       height:400,
                  animation: {
                duration: 0
         },
});
Si_No=Si_No+1;
}



