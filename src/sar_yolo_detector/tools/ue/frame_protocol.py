"""PNG completion checks for UE exports, including trailing allocation padding."""
import struct


def png_complete(path):
    try:
        with path.open('rb') as stream:
            if stream.read(8)!=b'\x89PNG\r\n\x1a\n':return False
            while True:
                header=stream.read(8)
                if len(header)!=8:return False
                length,kind=struct.unpack('>I4s',header)
                if length>64*1024*1024:return False
                stream.seek(length,1)
                crc=stream.read(4)
                if len(crc)!=4:return False
                if kind==b'IEND':return length==0 and crc==b'\xaeB`\x82'
    except (OSError,ValueError):return False


def image_complete(path):
    if path.suffix.lower() not in ('.jpg','.jpeg'):return png_complete(path)
    try:
        with path.open('rb') as stream:
            if stream.read(2)!=b'\xff\xd8':return False
            stream.seek(-2,2)
            return stream.read(2)==b'\xff\xd9'
    except (OSError,ValueError):return False


def claim_request(state,request,native=False):
    """Native captures must never re-arm a persisted legacy file command."""
    if native:return True
    key=(request.get('session'),request.get('id'))
    if key in (state.get('last_id'),state.get('startup_ignored_key')):return False
    state['last_id']=key
    return True
