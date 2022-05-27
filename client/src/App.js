import React, { useEffect, useState } from 'react'

const  App = () => {
  const [data,setData] = useState({});

  useEffect(()=>{
    // getLogData().then(data => {
    //   const date = new Date(parseInt(data.timestamp)*1000);
    //   setData({status: data.status, timestamp:date});
    // });
    const date = new Date(parseInt("1653405062")*1000);
    const mockData = {status: true, timestamp: date.toLocaleString()};
    setData(mockData);
  },[]);

  return (
    <div style={{display:'flex',alignItems:'center' ,flexFlow:'column'}}>
      <h1>Touchless Door Locking</h1>
      <h2>DOOR STATUS</h2>
      <p style={{fontSize:'x-large',color:data.status? 'green':'red'}}>{data.status? 'unlocked' : 'locked'}</p>
      <h2>LATEST LOG</h2>
      <p>{data.timestamp}</p>
    </div>
  );
}

export default App;
