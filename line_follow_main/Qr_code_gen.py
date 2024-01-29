import segno

for i in range(10):
    qrcode = segno.make_qr(str(i))
    qrcode.save(
        "qr_codes/"+str(i)+".png",
        scale=20
    )