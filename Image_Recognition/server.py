from fastapi import FastAPI, UploadFile
import uvicorn

app = FastAPI()

@app.get("/status")
def status():
    return {"status": "ok"}

@app.post("/image")
async def image(file: UploadFile):
    # Always return Right Arrow for now (demo)
    return {"image_id": "38"}

@app.get("/stitch")
def stitch():
    return {"message": "Images stitched!"}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
