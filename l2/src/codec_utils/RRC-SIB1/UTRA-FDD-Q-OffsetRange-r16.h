/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UTRA_FDD_Q_OffsetRange_r16_H_
#define	_UTRA_FDD_Q_OffsetRange_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum UTRA_FDD_Q_OffsetRange_r16 {
	UTRA_FDD_Q_OffsetRange_r16_dB_24	= 0,
	UTRA_FDD_Q_OffsetRange_r16_dB_22	= 1,
	UTRA_FDD_Q_OffsetRange_r16_dB_20	= 2,
	UTRA_FDD_Q_OffsetRange_r16_dB_18	= 3,
	UTRA_FDD_Q_OffsetRange_r16_dB_16	= 4,
	UTRA_FDD_Q_OffsetRange_r16_dB_14	= 5,
	UTRA_FDD_Q_OffsetRange_r16_dB_12	= 6,
	UTRA_FDD_Q_OffsetRange_r16_dB_10	= 7,
	UTRA_FDD_Q_OffsetRange_r16_dB_8	= 8,
	UTRA_FDD_Q_OffsetRange_r16_dB_6	= 9,
	UTRA_FDD_Q_OffsetRange_r16_dB_5	= 10,
	UTRA_FDD_Q_OffsetRange_r16_dB_4	= 11,
	UTRA_FDD_Q_OffsetRange_r16_dB_3	= 12,
	UTRA_FDD_Q_OffsetRange_r16_dB_2	= 13,
	UTRA_FDD_Q_OffsetRange_r16_dB_1	= 14,
	UTRA_FDD_Q_OffsetRange_r16_dB0	= 15,
	UTRA_FDD_Q_OffsetRange_r16_dB1	= 16,
	UTRA_FDD_Q_OffsetRange_r16_dB2	= 17,
	UTRA_FDD_Q_OffsetRange_r16_dB3	= 18,
	UTRA_FDD_Q_OffsetRange_r16_dB4	= 19,
	UTRA_FDD_Q_OffsetRange_r16_dB5	= 20,
	UTRA_FDD_Q_OffsetRange_r16_dB6	= 21,
	UTRA_FDD_Q_OffsetRange_r16_dB8	= 22,
	UTRA_FDD_Q_OffsetRange_r16_dB10	= 23,
	UTRA_FDD_Q_OffsetRange_r16_dB12	= 24,
	UTRA_FDD_Q_OffsetRange_r16_dB14	= 25,
	UTRA_FDD_Q_OffsetRange_r16_dB16	= 26,
	UTRA_FDD_Q_OffsetRange_r16_dB18	= 27,
	UTRA_FDD_Q_OffsetRange_r16_dB20	= 28,
	UTRA_FDD_Q_OffsetRange_r16_dB22	= 29,
	UTRA_FDD_Q_OffsetRange_r16_dB24	= 30
} e_UTRA_FDD_Q_OffsetRange_r16;

/* UTRA-FDD-Q-OffsetRange-r16 */
typedef long	 UTRA_FDD_Q_OffsetRange_r16_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_UTRA_FDD_Q_OffsetRange_r16_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_UTRA_FDD_Q_OffsetRange_r16;
extern const asn_INTEGER_specifics_t asn_SPC_UTRA_FDD_Q_OffsetRange_r16_specs_1;
asn_struct_free_f UTRA_FDD_Q_OffsetRange_r16_free;
asn_struct_print_f UTRA_FDD_Q_OffsetRange_r16_print;
asn_constr_check_f UTRA_FDD_Q_OffsetRange_r16_constraint;
xer_type_decoder_f UTRA_FDD_Q_OffsetRange_r16_decode_xer;
xer_type_encoder_f UTRA_FDD_Q_OffsetRange_r16_encode_xer;
per_type_decoder_f UTRA_FDD_Q_OffsetRange_r16_decode_uper;
per_type_encoder_f UTRA_FDD_Q_OffsetRange_r16_encode_uper;
per_type_decoder_f UTRA_FDD_Q_OffsetRange_r16_decode_aper;
per_type_encoder_f UTRA_FDD_Q_OffsetRange_r16_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _UTRA_FDD_Q_OffsetRange_r16_H_ */
#include <asn_internal.h>
