var sno_global = 1

function table_update(fromstation,tostation,serial,id){
 var $table =$('#table')
$table.bootstrapTable('append', randomData(fromstation,tostation,serial,id))
$table.bootstrapTable('scrollTo', 'bottom')

}

function randomData(fromstation,tostation,serial,id) {

    var rows = []

      rows.push({
        sno: sno_global,
        id: id,
        from: fromstation,
        to: tostation,
        status: "Active",
        agv: "AGV_"+serial
      })
    sno_global = sno_global+1
    return rows

  }


